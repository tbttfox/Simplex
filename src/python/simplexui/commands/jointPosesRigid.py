# Copyright 2016, Blur Studio
#
# This file is part of Simplex.
#
# Simplex is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Simplex is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with Simplex.  If not, see <http://www.gnu.org/licenses/>.
"""A layered joint pose solve with its own rigid fitter instead of Dem Bones

This is the solve that `jointPoses.solveLayeredPoses` used before it switched to
Dem Bones. It takes the same jobs and passes, but fits each joint with a weighted
rigid fit that supports:

* Rotation regularization toward the starting local rotations (``rotReg``),
  with a ``rotRadius`` floor so joints that move a small cluster of points don't
  read stretching as a big rotation
* A pull toward the prior pose where a joint has little influence (``priorReg``)
* The per-joint rotate/translate flags. A joint that can't translate rotates
  around its pivot

The rotations come from an iterative extraction (Muller et al. 2016) that starts
from the current rotation, so it never flips to a far away answer
"""

from __future__ import annotations

import multiprocessing
import os
import pickle
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from multiprocessing import shared_memory
from typing import Sequence

import numpy as np
import scipy.sparse
from scipy.spatial.transform import Rotation

from .jointPoses import (
    ProgressCallback,
    SolveJob,
    SolvePass,
    _jointOrder,
    _progress,
    invAffine,
    localMats,
    toHomogeneous,
    worldMats,
)


def _extractRotation(a: np.ndarray, r: np.ndarray, iterations: int = 8) -> np.ndarray:
    """Find the rotations that maximize ``trace(R.T @ A)``

    This is the iterative method from "A Robust Method to Extract the Rotational
    Part of Deformations" (Muller et al. 2016). Unlike an SVD, it walks
    continuously from the starting rotation, so it never jumps to a far away
    (flipped) answer

    Parameters
    ----------
    a : np.ndarray
        A (S, 3, 3) array of column-convention covariance matrices
    r : np.ndarray
        The (S, 3, 3) column-convention rotations to start from
    iterations : int, optional
        The max number of iterations

    Returns
    -------
    : np.ndarray
        The (S, 3, 3) column-convention rotations
    """
    for _ in range(iterations):
        # The sum of the cross products of the columns of r and a is the
        # skew-symmetric part of a @ r.T, and their dot products are its trace
        m = a @ np.swapaxes(r, -1, -2)
        num = np.stack(
            (m[:, 2, 1] - m[:, 1, 2], m[:, 0, 2] - m[:, 2, 0], m[:, 1, 0] - m[:, 0, 1]),
            axis=-1,
        )
        omega = num / (np.abs(np.trace(m, axis1=-2, axis2=-1)) + 1.0e-12)[:, None]
        if np.linalg.norm(omega, axis=-1).max() < 1.0e-9:
            break
        r = Rotation.from_rotvec(omega).as_matrix() @ r
    return r


@dataclass
class _Moments:
    """The weighted moments of the points a free joint moves

    The points don't change during a pass, so these get built once, and each fit
    only has to deal with the moments of its targets. The data is weighted by the
    squared skin weights (``D``), and the prior by ``priorReg`` times the skin
    weights (``P``). All of them are (S, ...) arrays in rest space

    Attributes
    ----------
    sumD, sumP : float
        The total weights
    firstD, firstP : np.ndarray
        The (S, 3) weighted sums of the points
    secondD, secondP : np.ndarray
        The (S, 3, 3) weighted sums of the outer products of the points
    """

    sumD: float
    sumP: float
    firstD: np.ndarray
    firstP: np.ndarray
    secondD: np.ndarray
    secondP: np.ndarray


def _moments(h: np.ndarray, vertWeights: np.ndarray, priorReg: float) -> _Moments:
    """Get the moments of the (S, n, 4) homogeneous points that are premultiplied by
    their (n,) skin weights"""
    wPts = h[..., :3]
    wPtsT = np.swapaxes(wPts, -1, -2)
    return _Moments(
        sumD=float(vertWeights @ vertWeights),
        sumP=priorReg * float(vertWeights.sum()),
        firstD=vertWeights @ wPts,
        firstP=priorReg * wPts.sum(axis=1),
        secondD=wPtsT @ wPts,
        secondP=priorReg * (wPtsT @ (wPts / vertWeights[:, None])),
    )


def _outer(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """The outer products of two (S, 3) arrays"""
    return a[:, :, None] * b[:, None, :]


def _fitRigid(
    m: _Moments,
    dataSum: np.ndarray,
    dataCross: np.ndarray,
    curMat: np.ndarray,
    priorMat: np.ndarray,
    lockMat: np.ndarray,
    rotate: bool,
    translate: bool,
    rotReg: float,
    rotRadius: float,
    weightSum: float,
    pivot: np.ndarray,
) -> np.ndarray:
    """Fit rigid row-convention matrices moving the points onto their targets

    Every point ``x`` with skin weight ``w`` has a target ``r`` that ``w * x @ M``
    should match, which is the same as ``x @ M`` matching ``r / w`` with a weight of
    ``w**2``. Each point is also pulled toward ``x @ priorMat`` with a weight of
    ``priorReg * w``, so that mostly affects points that the joint barely
    influences. All of that only needs the moments of the points and targets

    Parameters
    ----------
    m : _Moments
        The moments of the points
    dataSum : np.ndarray
        The (S, 3) sums of ``w * r``
    dataCross : np.ndarray
        The (S, 3, 3) sums of ``w * outer(r, x)``
    curMat : np.ndarray
        The (S, 4, 4) current matrices. Their rotation is the starting point of the fit
    priorMat : np.ndarray
        The (S, 4, 4) matrices that the regularization pulls toward
    lockMat : np.ndarray
        The (S, 4, 4) matrices that define what stays locked when rotate or
        translate is off
    rotate : bool
        Whether the matrices can rotate
    translate : bool
        Whether the matrices can translate. If not, they rotate around the pivot
    rotReg : float
        The rotation regularization
    rotRadius : float
        Regularize the rotations as if every point were at least this far from the
        center of rotation. Otherwise a joint that only moves a small cluster of
        points will take any stretching of the cluster as a big rotation
    weightSum : float
        The sum of the skin weights
    pivot : np.ndarray
        The (3,) rest position of the joint

    Returns
    -------
    : np.ndarray
        The (S, 4, 4) fit matrices
    """
    priorRot, priorTrn = priorMat[:, :3, :3], priorMat[:, 3, :3]
    sumW = m.sumD + m.sumP
    srcSum = m.firstD + m.firstP
    tgtSum = dataSum + (m.firstP[:, None] @ priorRot)[:, 0] + m.sumP * priorTrn

    if translate:
        ctrSrc = srcSum / sumW
        ctrTgt = tgtSum / sumW
    else:
        ctrSrc = np.broadcast_to(pivot, srcSum.shape)
        ctrTgt = pivot @ lockMat[:, :3, :3] + lockMat[:, 3, :3]

    ret = lockMat.copy()
    if rotate:
        # The covariances of the centered points, in column-convention so their
        # rotation is R.T
        cross = (
            dataCross
            + np.swapaxes(priorRot, -1, -2) @ m.secondP
            + _outer(priorTrn, m.firstP)
        )
        cov = (
            cross
            - _outer(tgtSum, ctrSrc)
            - _outer(ctrTgt, srcSum)
            + sumW * _outer(ctrTgt, ctrSrc)
        )
        covSrc = (
            m.secondD
            + m.secondP
            - _outer(srcSum, ctrSrc)
            - _outer(ctrSrc, srcSum)
            + sumW * _outer(ctrSrc, ctrSrc)
        )
        # This is weighted linearly by the skin weights while the data is weighted
        # by their square, so joints with little influence are harder to rotate
        iso = np.eye(3) * (rotRadius * rotRadius * weightSum)
        cov = cov + rotReg * (np.swapaxes(priorRot, -1, -2) @ (covSrc + iso))

        rot = _extractRotation(cov, np.swapaxes(curMat[:, :3, :3], -1, -2))
        ret[:, :3, :3] = np.swapaxes(rot, -1, -2)
    ret[:, 3, :3] = ctrTgt - (ctrSrc[:, None, :] @ ret[:, :3, :3])[:, 0]
    return ret


# The smallest chunk of targets worth giving to its own process
_MIN_CHUNK = 8

# The data every chunk shares, set in each worker process by `_initWorker`
_SHARED: dict = {}


@dataclass
class _PassSetup:
    """The parts of a pass that don't depend on the shapes being solved, so they
    can be shared by every chunk

    Attributes
    ----------
    free : [int, ...]
        The joints that get solved, parents first
    carrier : [int, ...]
        For every joint, the free joint that carries it, or -1
    carriedBy : {int: [int, ...]}
        The joints each free joint carries, including itself
    supports : {int: np.ndarray}
        The indices of the vertices each free joint moves
    positions : {int: {int: np.ndarray}}
        For each free joint, where each carried joint's vertices are in its support
    vertWeights : {int: np.ndarray}
        The total weight each free joint has on each vertex of its support
    """

    free: list[int]
    carrier: list[int]
    carriedBy: dict[int, list[int]]
    supports: dict[int, np.ndarray]
    positions: dict[int, dict[int, np.ndarray]]
    vertWeights: dict[int, np.ndarray]


def _jointVerts(
    restVerts: np.ndarray, weights: np.ndarray
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Get the indices of the vertices each joint influences, and their homogeneous
    rest positions premultiplied by the joint's weights

    Skin weights only have a few influences per vertex, so working with these is
    much faster than working with the full (N, J) weights
    """
    jIdx, vIdx = np.nonzero(weights.T)
    splits = np.searchsorted(jIdx, np.arange(1, weights.shape[1]))
    restH = toHomogeneous(restVerts)
    ret = []
    for k, idx in enumerate(np.split(vIdx, splits)):
        ret.append((idx, restH[idx] * weights[idx, k, None]))
    return ret


def _skinMatrix(
    jointVerts: list[tuple[np.ndarray, np.ndarray]], numVerts: int
) -> scipy.sparse.csr_array:
    """Build the sparse (N, J * 4) matrix that linear blend skins the rest points

    Multiplying it by the (J * 4, 3) stacked skinning matrices of a pose gives the
    skinned points. Every row has the homogeneous rest position of its vertex,
    premultiplied by the vertex's weight for each joint that influences it, which
    is exactly what `_jointVerts` gives
    """
    rows = np.concatenate([np.repeat(idx, 4) for idx, _ in jointVerts])
    cols = np.concatenate(
        [
            np.tile(np.arange(4) + 4 * k, len(idx))
            for k, (idx, _) in enumerate(jointVerts)
        ]
    )
    vals = np.concatenate([pts.ravel() for _, pts in jointVerts])
    return scipy.sparse.csr_array(
        (vals, (rows, cols)), shape=(numVerts, len(jointVerts) * 4)
    )


def _skinPoints(skinMat: scipy.sparse.csr_array, skin: np.ndarray) -> np.ndarray:
    """Linear blend skin the rest points with the output of `_skinMatrix`

    Returns
    -------
    : np.ndarray
        The (S, N, 3) skinned points
    """
    numShapes, numJoints = skin.shape[:2]
    mats = skin[..., :3].transpose(1, 2, 0, 3).reshape(numJoints * 4, numShapes * 3)
    out = (skinMat @ mats).reshape(-1, numShapes, 3)
    return np.ascontiguousarray(out.transpose(1, 0, 2))


def _passSetup(
    solvePass: SolvePass,
    parents: Sequence[int],
    order: Sequence[int],
    jointVerts: list[tuple[np.ndarray, np.ndarray]],
    numVerts: int,
    minWeight: float,
) -> _PassSetup:
    """Figure out which joints a pass solves, and what each of them carries"""
    free = [j for j in order if j in solvePass.joints]
    while True:
        freeSet = set(free)
        carrier = [-1] * len(parents)
        for k in order:
            p = parents[k]
            carrier[k] = k if k in freeSet else (carrier[p] if p >= 0 else -1)
        carriedBy: dict[int, list[int]] = {j: [] for j in free}
        for k, c in enumerate(carrier):
            if c >= 0:
                carriedBy[c].append(k)

        # Joints with barely any influence can't be solved reliably, so drop them
        # and let their ancestors carry them instead
        totals = {}
        for j in free:
            tot = np.zeros(numVerts)
            for k in carriedBy[j]:
                idx, pts = jointVerts[k]
                tot[idx] += pts[:, 3]
            totals[j] = tot
        keep = [j for j in free if totals[j].max() >= minWeight]
        if len(keep) == len(free):
            break
        free = keep

    supports, positions, vertWeights = {}, {}, {}
    for j in free:
        vIdx = np.nonzero(totals[j])[0]
        pos = np.full(numVerts, -1)
        pos[vIdx] = np.arange(len(vIdx))
        supports[j] = vIdx
        positions[j] = {k: pos[jointVerts[k][0]] for k in carriedBy[j]}
        vertWeights[j] = totals[j][vIdx]
    return _PassSetup(free, carrier, carriedBy, supports, positions, vertWeights)


def _solvePass(
    targetVerts: np.ndarray,
    jointMats: np.ndarray,
    invRest: np.ndarray,
    parents: Sequence[int],
    jointVerts: list[tuple[np.ndarray, np.ndarray]],
    skinMat: scipy.sparse.csr_array,
    setup: _PassSetup,
    local: np.ndarray,
    priorLocal: np.ndarray,
    solvePass: SolvePass,
    iterations: int,
    rotRegScale: float,
    rotRadius: float,
    priorReg: float,
) -> np.ndarray:
    """Run a single pass of the layered solve on a chunk of shapes

    Every joint that isn't solved in this pass keeps its local transform, so it
    gets carried along by its closest solved ancestor. That means each solved joint
    is fit to the points skinned to it *and* to all the joints it carries. Joints
    with no solved ancestor keep their world transforms

    Returns
    -------
    : np.ndarray
        The updated (S, J, 4, 4) local matrices
    """
    free, carrier = setup.free, setup.carrier
    if not free:
        return local
    numShapes = len(local)
    skin = invRest[None] @ worldMats(local, parents)

    # For every free joint, build the homogeneous points it moves (premultiplied by
    # the skin weights). The carried joints' skin matrices are K @ skin[free] where K
    # stays constant, so those points can be pre-transformed by K
    ptsH, wPtsT, moments, tgts = {}, {}, {}, {}
    for j in free:
        vIdx = setup.supports[j]
        invJ = invAffine(skin[:, j])
        h = np.zeros((numShapes, len(vIdx), 4))
        for k in setup.carriedBy[j]:
            pos, pts = setup.positions[j][k], jointVerts[k][1]
            if k == j:
                h[:, pos] += pts[None]
            else:
                h[:, pos] += pts[None] @ (skin[:, k] @ invJ)
        ptsH[j] = h
        wPtsT[j] = np.ascontiguousarray(np.swapaxes(h[..., :3], -1, -2))
        moments[j] = _moments(h, setup.vertWeights[j], priorReg)
        tgts[j] = targetVerts[:, vIdx]

    # The current skinned positions
    pred = _skinPoints(skinMat, skin)
    freeSkin = {j: skin[:, j].copy() for j in free}
    carrierOffset = {}
    for k, c in enumerate(carrier):
        if c >= 0 and k != c:
            carrierOffset[k] = skin[:, k] @ invAffine(skin[:, c])

    def skinOf(k):
        c = carrier[k]
        if c < 0:
            return skin[:, k]
        if c == k:
            return freeSkin[k]
        return carrierOffset[k] @ freeSkin[c]

    rotReg = solvePass.rotReg * rotRegScale
    for _ in range(iterations):
        for j in free:
            vIdx, vw, m = setup.supports[j], setup.vertWeights[j], moments[j]
            cur = freeSkin[j]
            curRot, curTrn = cur[:, :3, :3], cur[:, 3, :3]

            # Each point's target is what's left over once every other joint has
            # moved it, which is ``err + w * (x @ cur)`` for the current error. So
            # the target moments are the error's moments plus the points' moments
            # moved by cur
            err = tgts[j] - pred[:, vIdx]
            dataSum = vw @ err + (m.firstD[:, None] @ curRot)[:, 0] + m.sumD * curTrn
            dataCross = (
                np.swapaxes(wPtsT[j] @ err, -1, -2)
                + np.swapaxes(curRot, -1, -2) @ m.secondD
                + _outer(curTrn, m.firstD)
            )

            # Where this joint would be if it kept its prior local transform,
            # or the local transform it started the pass with
            p = parents[j]
            parentWorld = np.eye(4) if p < 0 else jointMats[p] @ skinOf(p)
            prior = invRest[j] @ priorLocal[:, j] @ parentWorld
            lock = invRest[j] @ local[:, j] @ parentWorld

            rotate, translate = solvePass.joints[j]
            new = _fitRigid(
                m,
                dataSum,
                dataCross,
                cur,
                prior,
                lock,
                rotate,
                translate,
                rotReg,
                rotRadius,
                float(vw.sum()),
                jointMats[j, 3, :3],
            )
            pred[:, vIdx] += ptsH[j] @ (new - cur)[..., :3]
            freeSkin[j] = new

    for k in range(len(parents)):
        if carrier[k] >= 0:
            skin[:, k] = skinOf(k)
    return localMats(jointMats[None] @ skin, parents)


def _solveChunk(
    shared: dict, jobIdxs: Sequence[int], prior: np.ndarray, tgts: np.ndarray
) -> np.ndarray:
    """Run all the passes on one chunk of targets

    Parameters
    ----------
    shared : dict
        The data that every chunk shares. See `solveLayeredPoses`
    jobIdxs : [int, ...]
        The jobs to run on this chunk, in order
    prior : np.ndarray
        The (S, J, 4, 4) local matrices to start from
    tgts : np.ndarray
        The (S, N, 3) absolute target positions

    Returns
    -------
    : np.ndarray
        The (S, J, 4, 4) world matrices of each joint for each target
    """
    parents = shared["parents"]
    local = prior.copy()
    for ji in jobIdxs:
        passes = shared["solveJobs"][ji].passes
        for solvePass, setup in zip(passes, shared["setups"][ji]):
            local = _solvePass(
                tgts,
                shared["jointMats"],
                shared["invRest"],
                parents,
                shared["jointVerts"],
                shared["skinMat"],
                setup,
                local,
                prior,
                solvePass,
                shared["iterations"],
                shared["rotRegScale"],
                shared["rotRadius"],
                shared["priorReg"],
            )
    return worldMats(local, parents)


def _attachMemory(name: str) -> shared_memory.SharedMemory:
    """Attach to shared memory that another process owns"""
    # Before python 3.13, attaching registers the memory with the resource tracker
    # on posix, which would delete it when this worker exits
    kwargs = {"track": False} if sys.version_info >= (3, 13) else {}
    return shared_memory.SharedMemory(name=name, **kwargs)


def _initWorker(dataName: str, dataSize: int, tgtName: str, tgtShape: tuple) -> None:
    """Load the shared data in a worker process. See `solveLayeredPoses`"""
    dataMem = _attachMemory(dataName)
    _SHARED.update(pickle.loads(dataMem.buf[:dataSize]))
    dataMem.close()
    # The targets get read in place, so keep their memory open
    tgtMem = _attachMemory(tgtName)
    _SHARED["targetMem"] = tgtMem
    _SHARED["targets"] = np.ndarray(tgtShape, dtype=np.float64, buffer=tgtMem.buf)


def _solveChunkInWorker(
    jobIdxs: Sequence[int], idxs: Sequence[int], prior: np.ndarray
) -> np.ndarray:
    """Run `_solveChunk` in a worker process with its shared data"""
    return _solveChunk(_SHARED, jobIdxs, prior, _SHARED["targets"][idxs])


def _spawnContext():
    """Get a multiprocessing context that starts fresh python processes

    Spawned processes run sys.executable, but inside Maya that's the Maya
    application itself, so they need to run mayapy instead
    """
    ctx = multiprocessing.get_context("spawn")
    exeDir, exeName = os.path.split(sys.executable)
    if os.path.splitext(exeName)[0].lower() == "maya":
        mayapy = os.path.join(exeDir, "mayapy.exe" if os.name == "nt" else "mayapy")
        if not os.path.isfile(mayapy):
            raise RuntimeError(f"Can't find mayapy to run the solve with: {mayapy}")
        ctx.set_executable(mayapy)
    return ctx


def solveLayeredPoses(
    restVerts: np.ndarray,
    targetVerts: np.ndarray,
    jointMats: np.ndarray,
    parents: Sequence[int],
    weights: np.ndarray,
    jobs: Sequence[SolveJob],
    shapeJobs: Sequence[Sequence[int]] | None = None,
    initLocal: np.ndarray | None = None,
    lockJoints: Sequence[int] | None = None,
    iterations: int = 6,
    rotRegScale: float = 0.5,
    rotRadius: float | None = None,
    priorReg: float = 0.05,
    minWeight: float = 0.3,
    chunkSize: int = 64,
    processes: int | None = None,
    progress: ProgressCallback | None = None,
) -> np.ndarray:
    """Solve joint poses in layers with fixed skin weights

    This roughly follows how the MetaHuman expression editor fits joints. Each job is
    a sequence of passes, and each pass solves a subset of the joints, starting from
    the result of the previous pass. Joints that aren't solved yet get carried along
    by their solved ancestors, so the early passes solve the big parent joints with
    the influence of their whole hierarchy, and the later passes refine their children.

    The rotations are regularized toward the starting local rotations, and joints
    are only allowed to make small moves in areas where they have little influence.
    That, plus starting each fit from the previous result, keeps the poses consistent
    so they blend well.

    Parameters
    ----------
    restVerts : np.ndarray
        The (N, 3) rest positions
    targetVerts : np.ndarray
        The (S, N, 3) absolute target positions
    jointMats : np.ndarray
        The (J, 4, 4) rest world matrices of the joints
    parents : [int, ...]
        The parent index of each joint, or -1
    weights : np.ndarray
        The (N, J) skin weights. These must be normalized
    jobs : [SolveJob, ...]
        The jobs that can be run
    shapeJobs : [[int, ...], ...], optional
        For each target, the indices of the jobs to run on it, in order.
        Defaults to running all the jobs on every target
    initLocal : np.ndarray, optional
        (S, J, 4, 4) local matrices to start from. The rotations are regularized
        toward these. Defaults to the rest pose
    lockJoints : [int, ...], optional
        Joints that are never solved, even if a pass includes them
    iterations : int, optional
        The number of times each pass loops over its joints. Defaults to 6
    rotRegScale : float, optional
        A multiplier on each pass's rotation regularization. Defaults to 0.5
    rotRadius : float, optional
        The rotations are regularized as if every point were at least this far from
        its joint's center of rotation. Defaults to 5% of the RMS distance of the
        rest points from their centroid
    priorReg : float, optional
        How much joints are held in place where they have little influence.
        Defaults to 0.05
    minWeight : float, optional
        Joints whose max weight (including the joints they carry) is below this
        don't get solved, so they just follow their parents. Defaults to 0.3
    chunkSize : int, optional
        The most targets to solve at once. Defaults to 64
    processes : int, optional
        How many worker processes solve chunks of targets in parallel. The chunks
        are made smaller if needed so every process gets one, as long as they
        have at least `_MIN_CHUNK` targets. Use 1 to solve in this process.
        Defaults to 16, or the number of CPUs if that's lower
    progress : callable, optional
        Called as ``progress(value, maximum, message)``. This is only ever called
        from this process's calling thread

    Returns
    -------
    : np.ndarray
        The (S, J, 4, 4) world matrices of each joint for each target
    """
    restVerts = np.asarray(restVerts, dtype=np.float64)
    targetVerts = np.asarray(targetVerts, dtype=np.float64)
    jointMats = np.asarray(jointMats, dtype=np.float64)
    weights = np.asarray(weights, dtype=np.float64)
    numShapes = len(targetVerts)
    if shapeJobs is None:
        shapeJobs = [range(len(jobs))] * numShapes
    if len(shapeJobs) != numShapes:
        raise ValueError("There must be a list of jobs for every target")

    restLocal = localMats(jointMats, parents)
    if initLocal is None:
        initLocal = np.broadcast_to(restLocal, (numShapes,) + restLocal.shape)
    invRest = invAffine(jointMats)
    order = _jointOrder(parents)
    locked = set(lockJoints or [])
    if rotRadius is None:
        modelSize = np.sqrt(
            ((restVerts - restVerts.mean(axis=0)) ** 2).sum(axis=1).mean()
        )
        rotRadius = 0.05 * modelSize

    # Don't let the passes touch locked joints
    solveJobs = []
    for job in jobs:
        passes = []
        for p in job.passes:
            jointDict = {
                j: rt for j, rt in p.joints.items() if j not in locked and any(rt)
            }
            passes.append(SolvePass(p.name, jointDict, p.rotReg))
        solveJobs.append(SolveJob(job.name, passes))

    # Targets with the same jobs get solved together. Every target is solved
    # independently, so the chunks can be made smaller to keep every process busy.
    # But each chunk has a fixed cost per joint, so they can't get too small
    if processes is None:
        processes = min(16, os.cpu_count() or 1)
    processes = max(1, processes)
    chunkSize = max(1, min(chunkSize, max(_MIN_CHUNK, -(-numShapes // processes))))
    groups: dict[tuple[int, ...], list[int]] = {}
    for s, jobIdxs in enumerate(shapeJobs):
        groups.setdefault(tuple(jobIdxs), []).append(s)
    chunks = [
        (jobIdxs, idxs[i : i + chunkSize])
        for jobIdxs, idxs in groups.items()
        for i in range(0, len(idxs), chunkSize)
    ]

    # Everything about the passes that doesn't depend on the shapes is shared by
    # every chunk, so build it once up front
    jointVerts = _jointVerts(restVerts, weights)
    setups = [
        [
            _passSetup(p, parents, order, jointVerts, len(restVerts), minWeight)
            for p in job.passes
        ]
        for job in solveJobs
    ]

    shared = dict(
        jointMats=jointMats,
        invRest=invRest,
        parents=parents,
        jointVerts=jointVerts,
        skinMat=_skinMatrix(jointVerts, len(restVerts)),
        solveJobs=solveJobs,
        setups=setups,
        iterations=iterations,
        rotRegScale=rotRegScale,
        rotRadius=rotRadius,
        priorReg=priorReg,
    )

    out = np.empty((numShapes,) + jointMats.shape)
    done = 0
    _progress(progress, done, numShapes, "Solving poses")
    if processes == 1 or len(chunks) <= 1:
        for jobIdxs, idxs in chunks:
            out[idxs] = _solveChunk(
                shared, jobIdxs, np.array(initLocal[idxs]), targetVerts[idxs]
            )
            done += len(idxs)
            _progress(progress, done, numShapes, "Solving poses")
        _progress(progress, numShapes, numShapes, "Done")
        return out

    # The fits make lots of small numpy calls, so threads spend most of their time
    # waiting on the GIL. Separate processes don't have that problem.
    # The workers start one at a time, and each one blocks until the last has read
    # its startup arguments. So the shared data and targets go in shared memory
    # instead, and the workers all start loading them at once
    data = pickle.dumps(shared, protocol=pickle.HIGHEST_PROTOCOL)
    dataMem = shared_memory.SharedMemory(create=True, size=len(data))
    tgtMem = shared_memory.SharedMemory(create=True, size=targetVerts.nbytes)
    try:
        dataMem.buf[: len(data)] = data
        tgtView = np.ndarray(targetVerts.shape, dtype=np.float64, buffer=tgtMem.buf)
        tgtView[:] = targetVerts
        del tgtView

        with ProcessPoolExecutor(
            max_workers=min(processes, len(chunks)),
            mp_context=_spawnContext(),
            initializer=_initWorker,
            initargs=(dataMem.name, len(data), tgtMem.name, targetVerts.shape),
        ) as pool:
            futures = {
                pool.submit(
                    _solveChunkInWorker, jobIdxs, idxs, np.array(initLocal[idxs])
                ): idxs
                for jobIdxs, idxs in chunks
            }
            for future in as_completed(futures):
                idxs = futures[future]
                out[idxs] = future.result()
                done += len(idxs)
                _progress(progress, done, numShapes, "Solving poses")
    finally:
        for mem in (dataMem, tgtMem):
            mem.close()
            mem.unlink()
    _progress(progress, numShapes, numShapes, "Done")
    return out
