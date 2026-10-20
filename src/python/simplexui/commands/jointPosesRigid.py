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
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .jointPoses import (
    ProgressCallback,
    SolveJob,
    SolvePass,
    _jointOrder,
    _progress,
    _rotToQuat,
    invAffine,
    localMats,
    toHomogeneous,
    worldMats,
)


def _quatToRot(q: np.ndarray) -> np.ndarray:
    """Convert xyzw quaternions to column-convention 3x3 rotation matrices"""
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    ret = np.empty(q.shape[:-1] + (3, 3))
    ret[..., 0, 0] = 1 - 2 * (y * y + z * z)
    ret[..., 0, 1] = 2 * (x * y - z * w)
    ret[..., 0, 2] = 2 * (x * z + y * w)
    ret[..., 1, 0] = 2 * (x * y + z * w)
    ret[..., 1, 1] = 1 - 2 * (x * x + z * z)
    ret[..., 1, 2] = 2 * (y * z - x * w)
    ret[..., 2, 0] = 2 * (x * z - y * w)
    ret[..., 2, 1] = 2 * (y * z + x * w)
    ret[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return ret


def _quatMul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Multiply xyzw quaternions so the result rotates by b, then by a"""
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ),
        axis=-1,
    )


def _extractRotation(a: np.ndarray, q: np.ndarray, iterations: int = 8) -> np.ndarray:
    """Find the rotations that maximize ``trace(R.T @ A)``

    This is the iterative method from "A Robust Method to Extract the Rotational
    Part of Deformations" (Muller et al. 2016). Unlike an SVD, it walks
    continuously from the starting rotation, so it never jumps to a far away
    (flipped) answer. And it doesn't make any LAPACK calls

    Parameters
    ----------
    a : np.ndarray
        A (..., 3, 3) array of column-convention covariance matrices
    q : np.ndarray
        A (..., 4) array of xyzw quaternions to start from
    iterations : int, optional
        The max number of iterations

    Returns
    -------
    : np.ndarray
        A (..., 4) array of xyzw quaternions
    """
    for _ in range(iterations):
        r = _quatToRot(q)
        # The sum of the cross products of the columns of r and a
        # np.cross has a lot of overhead, so write it out
        num = np.stack(
            (
                (r[..., 1, :] * a[..., 2, :] - r[..., 2, :] * a[..., 1, :]).sum(-1),
                (r[..., 2, :] * a[..., 0, :] - r[..., 0, :] * a[..., 2, :]).sum(-1),
                (r[..., 0, :] * a[..., 1, :] - r[..., 1, :] * a[..., 0, :]).sum(-1),
            ),
            axis=-1,
        )
        den = np.abs((r * a).sum(axis=(-1, -2))) + 1.0e-12
        omega = num / den[..., None]
        angle = np.linalg.norm(omega, axis=-1)
        if angle.max() < 1.0e-9:
            break
        with np.errstate(divide="ignore", invalid="ignore"):
            scale = np.where(angle > 1.0e-15, np.sin(angle / 2) / angle, 0.5)
        dq = np.concatenate(
            (omega * scale[..., None], np.cos(angle / 2)[..., None]), -1
        )
        q = _quatMul(dq, q)
        q /= np.linalg.norm(q, axis=-1, keepdims=True)
    return q


def _fitRigid(
    pts: np.ndarray,
    ptWeights: np.ndarray,
    targets: np.ndarray,
    curMat: np.ndarray,
    priorMat: np.ndarray,
    lockMat: np.ndarray,
    rotate: bool,
    translate: bool,
    rotReg: float,
    rotRadius: float,
    priorReg: float,
    pivot: np.ndarray,
) -> np.ndarray:
    """Fit a rigid row-convention matrix moving the points onto the targets

    Parameters
    ----------
    pts : np.ndarray
        The (S, n, 3) points to move
    ptWeights : np.ndarray
        The (n,) skin weight of each point
    targets : np.ndarray
        The (S, n, 3) positions that ``ptWeights * pts @ mat`` should match
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
    priorReg : float
        The strength of the pull toward the prior matrices. This is weighted
        linearly by the skin weights while the data is weighted by their square, so
        it mostly affects points that the joint barely influences
    pivot : np.ndarray
        The (3,) rest position of the joint

    Returns
    -------
    : np.ndarray
        The (S, 4, 4) fit matrices
    """
    # Minimizing |w * (x @ M) - r|^2 is the same as minimizing w^2 * |x @ M - r/w|^2
    dataW = ptWeights * ptWeights
    priorW = priorReg * ptWeights
    totW = dataW + priorW
    priorPts = pts @ priorMat[:, :3, :3] + priorMat[:, None, 3, :3]
    tgts = targets * ptWeights[None, :, None] + priorW[None, :, None] * priorPts
    tgts /= totW[None, :, None]

    sumW = totW.sum()
    if translate:
        ctrSrc = (totW @ pts) / sumW
        ctrTgt = (totW @ tgts) / sumW
    else:
        ctrSrc = np.broadcast_to(pivot, (len(pts), 3))
        ctrTgt = pivot @ lockMat[:, :3, :3] + lockMat[:, 3, :3]

    ret = lockMat.copy()
    if rotate:
        src = pts - ctrSrc[:, None, :]
        tgt = tgts - ctrTgt[:, None, :]
        # Build the covariance in column-convention so its rotation is R.T
        wSrc = src * totW[None, :, None]
        cov = np.swapaxes(tgt, -1, -2) @ wSrc
        covSrc = np.swapaxes(src, -1, -2) @ wSrc
        priorRot = np.swapaxes(priorMat[:, :3, :3], -1, -2)
        # This is weighted linearly by the skin weights while the data is weighted
        # by their square, so joints with little influence are harder to rotate
        iso = np.eye(3) * (rotRadius * rotRadius * ptWeights.sum())
        cov = cov + rotReg * (priorRot @ (covSrc + iso))

        startQ = _rotToQuat(np.swapaxes(curMat[:, :3, :3], -1, -2))
        rot = np.swapaxes(_quatToRot(_extractRotation(cov, startQ)), -1, -2)
        ret[:, :3, :3] = rot
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


def _skinSparse(
    jointVerts: list[tuple[np.ndarray, np.ndarray]], numVerts: int, skin: np.ndarray
) -> np.ndarray:
    """Linear blend skin the rest points with the output of `_jointVerts`

    Returns
    -------
    : np.ndarray
        The (S, N, 3) skinned points
    """
    out = np.zeros((len(skin), numVerts, 3))
    for k, (idx, pts) in enumerate(jointVerts):
        if len(idx):
            out[:, idx] += pts[None] @ skin[:, k, :, :3]
    return out


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
    numShapes, numVerts = len(local), targetVerts.shape[1]
    skin = invRest[None] @ worldMats(local, parents)

    # For every free joint, build the homogeneous points it moves (premultiplied by
    # the skin weights). The carried joints' skin matrices are K @ skin[free] where K
    # stays constant, so those points can be pre-transformed by K
    ptsH, pts3, tgts = {}, {}, {}
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
        pts3[j] = h[..., :3] / setup.vertWeights[j][None, :, None]
        tgts[j] = targetVerts[:, vIdx]

    # The current skinned positions
    pred = _skinSparse(jointVerts, numVerts, skin)
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
            vIdx, h = setup.supports[j], ptsH[j]
            vw = setup.vertWeights[j]
            cur = freeSkin[j]
            resid = tgts[j] - pred[:, vIdx] + h @ cur[..., :3]

            # Where this joint would be if it kept its prior local transform,
            # or the local transform it started the pass with
            p = parents[j]
            parentWorld = np.eye(4) if p < 0 else jointMats[p] @ skinOf(p)
            prior = invRest[j] @ priorLocal[:, j] @ parentWorld
            lock = invRest[j] @ local[:, j] @ parentWorld

            rotate, translate = solvePass.joints[j]
            new = _fitRigid(
                pts3[j],
                vw,
                resid,
                cur,
                prior,
                lock,
                rotate,
                translate,
                rotReg,
                rotRadius,
                priorReg,
                jointMats[j, 3, :3],
            )
            pred[:, vIdx] += h @ (new - cur)[..., :3]
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


def _initWorker(shared: dict) -> None:
    """Store the shared data in a worker process"""
    _SHARED.update(shared)


def _solveChunkInWorker(
    jobIdxs: Sequence[int], prior: np.ndarray, tgts: np.ndarray
) -> np.ndarray:
    """Run `_solveChunk` in a worker process with its shared data"""
    return _solveChunk(_SHARED, jobIdxs, prior, tgts)


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
    if processes == 1 or len(chunks) == 1:
        for jobIdxs, idxs in chunks:
            out[idxs] = _solveChunk(
                shared, jobIdxs, np.array(initLocal[idxs]), targetVerts[idxs]
            )
            done += len(idxs)
            _progress(progress, done, numShapes, "Solving poses")
    else:
        # The fits make lots of small numpy calls, so threads spend most of their
        # time waiting on the GIL. Separate processes don't have that problem, and
        # the shared data only gets sent to each of them once
        with ProcessPoolExecutor(
            max_workers=min(processes, len(chunks)),
            mp_context=_spawnContext(),
            initializer=_initWorker,
            initargs=(shared,),
        ) as pool:
            futures = {
                pool.submit(
                    _solveChunkInWorker,
                    jobIdxs,
                    np.array(initLocal[idxs]),
                    targetVerts[idxs],
                ): idxs
                for jobIdxs, idxs in chunks
            }
            for future in as_completed(futures):
                idxs = futures[future]
                out[idxs] = future.result()
                done += len(idxs)
                _progress(progress, done, numShapes, "Solving poses")
    _progress(progress, numShapes, numShapes, "Done")
    return out
