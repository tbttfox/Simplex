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
"""Solve smooth skin weights for fixed joint poses

Fitting every vertex's weights on its own, like Dem Bones does, lets neighboring
vertices pick different joints, which looks crunchy. This solves every vertex at
once, minimizing the skinning error of all the targets plus a smoothness term that
penalizes each joint's weights changing across the mesh's edges. The weights are
kept non-negative, normalized, and limited to a max number of influences.

Vertices can be ignored, so they don't have any data term. Their weights then
smoothly fill in from the vertices around them.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np
import scipy.sparse
import scipy.sparse.linalg

from .jointPoses import skinMatrices, toHomogeneous


def meshEdges(faces: Sequence[Sequence[int]]) -> np.ndarray:
    """Get the (E, 2) unique edges of a mesh, with the lower index first"""
    a = np.concatenate([np.asarray(f) for f in faces])
    b = np.concatenate([np.roll(f, -1) for f in faces])
    edges = np.unique(np.sort(np.stack((a, b), axis=1), axis=1), axis=0)
    return edges[edges[:, 0] != edges[:, 1]]


def _nearestJoints(
    restVerts: np.ndarray, jointMats: np.ndarray, joints: Sequence[int], count: int
) -> np.ndarray:
    """Get the (N, count) indices of each vertex's closest joints, closest first"""
    joints = np.asarray(joints, dtype=int)
    count = min(count, len(joints))
    dists = np.linalg.norm(
        restVerts[:, None, :] - jointMats[joints, 3, :3][None, :, :], axis=-1
    )
    near = np.argpartition(dists, count - 1, axis=1)[:, :count]
    near = np.take_along_axis(
        near, np.argsort(np.take_along_axis(dists, near, axis=1), axis=1), axis=1
    )
    return joints[near]


def _dataTerms(
    restVerts: np.ndarray,
    targetVerts: np.ndarray,
    skin: np.ndarray,
    cands: np.ndarray,
    chunkSize: int = 4096,
) -> tuple[np.ndarray, np.ndarray]:
    """Get the per-vertex normal equations of the skinning error

    Everything is relative to the rest position, so with normalized weights the
    error at a vertex is ``sum(w_k * d_k) - t``, where ``d_k`` is how candidate
    joint k moves the vertex in each target, and ``t`` is how the vertex moves

    Expanding the sums over the targets means everything comes from a few small
    per-joint sums of the skinning matrices ``M``, instead of moving every vertex by
    every candidate in every target:

    * ``sum(dot(xh @ M_k, xh @ M_l))`` is ``xh @ sum(M_k @ M_l.T) @ xh``
    * ``sum(dot(xh @ M_k, x))`` is ``dot(xh @ sum(M_k), x)``
    * ``sum(dot(xh @ M_k, target))`` is ``xh @ (A_k @ targets)``, where ``A_k`` has
      every target's ``M_k`` side by side

    Returns
    -------
    : np.ndarray
        The (N, K, K) sums of ``dot(d_k, d_l)``
    : np.ndarray
        The (N, K) sums of ``dot(d_k, t)``
    """
    numVerts, numCands = cands.shape
    numShapes, numJoints = skin.shape[:2]

    # Work around the mesh's center, so the sums don't lose precision when the
    # mesh is far from the origin
    center = restVerts.mean(axis=0)
    toCenter, fromCenter = np.eye(4), np.eye(4)
    toCenter[3, :3] = -center
    fromCenter[3, :3] = center
    skin3 = (fromCenter @ skin @ toCenter)[..., :3]
    rest = restVerts - center
    restH = toHomogeneous(rest)

    # (J, 4, S * 3) skinning matrices of every target side by side
    sideBySide = skin3.transpose(1, 2, 0, 3).reshape(numJoints, 4, numShapes * 3)
    flat = sideBySide.reshape(numJoints * 4, numShapes * 3)
    pairs = (flat @ flat.T).reshape(numJoints, 4, numJoints, 4).transpose(0, 2, 1, 3)
    sums = skin3.sum(axis=0)

    # How far each candidate moves each vertex, summed over the targets
    moved = np.einsum("na,nkac->nkc", restH, sums[cands])
    movedDotRest = moved @ rest[:, :, None]
    restSq = numShapes * (rest * rest).sum(axis=1)

    gram = np.empty((numVerts, numCands, numCands))
    for start in range(0, numVerts, chunkSize):
        sl = slice(start, start + chunkSize)
        c = cands[sl]
        quad = pairs[c[:, :, None], c[:, None, :]]
        gram[sl] = np.einsum("na,nklab,nb->nkl", restH[sl], quad, restH[sl])
    gram -= movedDotRest + np.swapaxes(movedDotRest, -1, -2)
    gram += restSq[:, None, None]

    # The targets of every vertex side by side, then each joint's term for the
    # vertices it's a candidate of
    tgts = (targetVerts - center).transpose(1, 0, 2).reshape(numVerts, numShapes * 3)
    rhs = np.empty((numVerts, numCands))
    for j in np.unique(cands):
        verts, slots = np.nonzero(cands == j)
        moves = tgts[verts] @ sideBySide[j].T
        rhs[verts, slots] = (moves * restH[verts]).sum(axis=1)
    rhs -= movedDotRest[..., 0]
    rhs -= (rest * tgts.reshape(numVerts, numShapes, 3).sum(axis=1)).sum(axis=1)[
        :, None
    ]
    rhs += restSq[:, None]
    return gram, rhs


def _smoothnessMatrix(edges: np.ndarray, cands: np.ndarray) -> scipy.sparse.csr_array:
    """Build the matrix of the smoothness energy over every (vertex, candidate) slot

    The energy is the sum over the edges and joints of the squared difference of
    the joint's weights at the two ends of the edge. A joint that isn't one of the
    candidates of a vertex has a weight of 0 there, so the weights fall off to 0
    where a joint stops being a candidate
    """
    numVerts, numCands = cands.shape
    src = np.concatenate((edges[:, 0], edges[:, 1]))
    dst = np.concatenate((edges[:, 1], edges[:, 0]))

    # Every slot of the source vertex of every directed edge
    srcSlot = (src[:, None] * numCands + np.arange(numCands)).ravel()
    joint = cands[src].ravel()
    dstVert = np.repeat(dst, numCands)

    # Find the slot with the same joint at the other end of the edge
    match = cands[dstVert] == joint[:, None]
    found = match.any(axis=1)
    dstSlot = dstVert * numCands + match.argmax(axis=1)

    rows = np.concatenate((srcSlot, srcSlot[found]))
    cols = np.concatenate((srcSlot, dstSlot[found]))
    vals = np.concatenate((np.ones(len(srcSlot)), -np.ones(found.sum())))
    size = numVerts * numCands
    return scipy.sparse.csr_array((vals, (rows, cols)), shape=(size, size))


def _sumZeroBasis(size: int) -> np.ndarray:
    """Get a (size, size - 1) orthonormal basis of the vectors that sum to 0"""
    # The last size - 1 left singular vectors of a column of ones are orthogonal to it
    return np.linalg.svd(np.ones((size, 1)))[0][:, 1:]


def _solveActive(
    quad: scipy.sparse.csr_array,
    rhs: np.ndarray,
    active: np.ndarray,
    blockDiag: np.ndarray,
    guess: np.ndarray | None = None,
    tolerance: float = 1.0e-8,
) -> np.ndarray:
    """Minimize ``w.T @ quad @ w - 2 * rhs @ w`` so each vertex's weights sum to 1,
    with the inactive slots held at 0

    Each vertex's weights are written as an even split between its active slots,
    plus a combination of an orthonormal basis of the vectors that sum to 0. That
    leaves an unconstrained positive definite system that conjugate gradients
    solves, preconditioned by inverting each vertex's block

    Parameters
    ----------
    quad : scipy.sparse.csr_array
        The (N * K, N * K) quadratic term
    rhs : np.ndarray
        The (N * K,) linear term
    active : np.ndarray
        The (N, K) slots that can be non-zero. Every vertex needs at least one
    blockDiag : np.ndarray
        The (N, K, K) blocks of quad for each vertex
    guess : np.ndarray, optional
        (N, K) weights to start from
    tolerance : float, optional
        The relative residual to stop at

    Returns
    -------
    : np.ndarray
        The (N, K) weights
    """
    numVerts, numCands = active.shape
    counts = active.sum(axis=1)
    # The active slots of each vertex first
    slots = np.argsort(~active, axis=1, kind="stable")
    colStart = np.concatenate(([0], np.cumsum(counts - 1)))
    numCols = int(colStart[-1])

    even = np.zeros((numVerts, numCands))
    rows, cols, vals = [], [], []
    groups = []
    for size in np.unique(counts):
        verts = np.flatnonzero(counts == size)
        flatSlots = verts[:, None] * numCands + slots[verts, :size]
        even[verts[:, None], slots[verts, :size]] = 1.0 / size
        if size == 1:
            continue
        basis = _sumZeroBasis(size)
        vertCols = colStart[verts, None] + np.arange(size - 1)
        rows.append(np.repeat(flatSlots, size - 1, axis=1).ravel())
        cols.append(np.tile(vertCols, (1, size)).ravel())
        vals.append(np.broadcast_to(basis.ravel(), (len(verts), basis.size)).ravel())

        # Invert each vertex's block in the reduced space to precondition
        sub = blockDiag[
            verts[:, None, None], slots[verts, :size, None], slots[verts, None, :size]
        ]
        reduced = basis.T @ sub @ basis
        groups.append((vertCols, np.linalg.inv(reduced)))

    even = even.ravel()
    if numCols == 0:
        return even.reshape(numVerts, numCands)
    basis = scipy.sparse.csr_array(
        (np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
        shape=(numVerts * numCands, numCols),
    )
    reducedQuad = (basis.T @ quad @ basis).tocsr()
    reducedRhs = basis.T @ (rhs - quad @ even)

    def precondition(x):
        ret = np.empty_like(x)
        for vertCols, inv in groups:
            ret[vertCols] = (inv @ x[vertCols][..., None])[..., 0]
        return ret

    x0 = None if guess is None else basis.T @ (guess.ravel() - even)
    sol, info = scipy.sparse.linalg.cg(
        reducedQuad,
        reducedRhs,
        x0=x0,
        rtol=tolerance,
        maxiter=5000,
        M=scipy.sparse.linalg.LinearOperator(
            reducedQuad.shape, matvec=precondition, dtype=np.float64
        ),
    )
    if info > 0:
        print(f"The weight solve didn't fully converge after {info} iterations")
    return (even + basis @ sol).reshape(numVerts, numCands)


def solveSmoothWeights(
    restVerts: np.ndarray,
    targetVerts: np.ndarray,
    jointMats: np.ndarray,
    poseMats: np.ndarray,
    faces: Sequence[Sequence[int]],
    joints: Sequence[int] | None = None,
    maxInfluences: int = 4,
    candidates: int = 8,
    smoothness: float = 1.0,
    ignoreVerts: Sequence[int] | None = None,
    maxSolves: int = 8,
) -> tuple[np.ndarray, float]:
    """Solve smooth skin weights for fixed joint poses

    Parameters
    ----------
    restVerts : np.ndarray
        The (N, 3) rest positions
    targetVerts : np.ndarray
        The (S, N, 3) absolute target positions
    jointMats : np.ndarray
        The (J, 4, 4) rest world matrices of the joints
    poseMats : np.ndarray
        The (S, J, 4, 4) posed world matrices of the joints for each target
    faces : [[int, ...], ...]
        The mesh faces
    joints : [int, ...], optional
        The joints that can have weights. Defaults to all of them
    maxInfluences : int, optional
        The max number of joints per vertex. Defaults to 4
    candidates : int, optional
        How many of each vertex's closest joints it can pick its influences from.
        Defaults to 8
    smoothness : float, optional
        How strongly the weights are smoothed. It's relative to the average size
        of the data term, so 1.0 weights the smoothness about the same as the fit.
        Defaults to 1.0
    ignoreVerts : [int, ...], optional
        Vertices that the targets don't apply to. Their weights only come from
        smoothing the weights around them
    maxSolves : int, optional
        The max number of times to re-solve while removing negative weights and
        extra influences. Defaults to 8

    Returns
    -------
    : np.ndarray
        The (N, J) skin weights
    : float
        The root mean squared error of the fit, not including the ignored vertices
    """
    restVerts = np.asarray(restVerts, dtype=np.float64)
    targetVerts = np.asarray(targetVerts, dtype=np.float64)
    jointMats = np.asarray(jointMats, dtype=np.float64)
    numVerts, numJoints = len(restVerts), len(jointMats)
    if joints is None:
        joints = range(numJoints)
    keep = np.ones(numVerts, dtype=bool)
    if ignoreVerts is not None:
        keep[np.asarray(ignoreVerts, dtype=int)] = False

    skin = skinMatrices(jointMats, poseMats)
    cands = _nearestJoints(restVerts, jointMats, joints, candidates)
    numCands = cands.shape[1]
    gram, rhs = _dataTerms(restVerts, targetVerts, skin, cands)
    gram[~keep] = 0.0
    rhs[~keep] = 0.0

    # Scale the smoothness by the size of the data term, so it doesn't depend on
    # the units or the number of targets
    diag = np.trace(gram, axis1=-2, axis2=-1)[keep] / numCands
    scale = float(diag.mean()) if len(diag) and diag.mean() > 0 else 1.0

    size = numVerts * numCands
    blocks = np.arange(size).reshape(numVerts, numCands)
    data = scipy.sparse.csr_array(
        (
            gram.ravel(),
            (
                np.repeat(blocks, numCands, axis=1).ravel(),
                np.tile(blocks, (1, numCands)).ravel(),
            ),
        ),
        shape=(size, size),
    )
    edges = meshEdges(faces)
    smooth = _smoothnessMatrix(edges, cands)
    # A tiny pull toward 0 keeps the system solvable for ignored vertices that
    # aren't connected to anything
    tiny = scale * 1.0e-9
    quad = (
        data + (smoothness * scale) * smooth + scipy.sparse.eye_array(size) * tiny
    ).tocsr()
    rhs = rhs.ravel()

    # Each vertex's block of quad. The smoothness only adds the vertex's number of
    # edges to the diagonal
    degree = np.bincount(edges.ravel(), minlength=numVerts)
    blockDiag = gram + (smoothness * scale * degree + tiny)[:, None, None] * np.eye(
        numCands
    )

    # Remove negative weights, then any extra influences, and re-solve until
    # there aren't any of either
    active = np.ones((numVerts, numCands), dtype=bool)
    w = None
    for _ in range(maxSolves):
        w = _solveActive(quad, rhs, active, blockDiag, guess=w)
        neg = active & (w < 0.0)
        if neg.any():
            active &= ~neg
            continue
        over = active.sum(axis=1) > maxInfluences
        if not over.any():
            break
        ranks = np.argsort(np.argsort(-np.where(active, w, -np.inf), axis=1), axis=1)
        active &= ~(over[:, None] & (ranks >= maxInfluences))

    w = np.where(active, np.maximum(w, 0.0), 0.0)
    w /= w.sum(axis=1, keepdims=True)
    weights = np.zeros((numVerts, numJoints))
    np.add.at(weights, (np.arange(numVerts)[:, None], cands), w)

    return weights, skinRmse(restVerts, targetVerts, weights, skin, ignoreVerts)


def smoothWeights(
    weights: np.ndarray,
    faces: Sequence[Sequence[int]],
    iterations: int = 3,
    maxInfluences: int = 4,
    strength: float = 0.5,
) -> np.ndarray:
    """Smooth skin weights across a mesh, like flood smoothing them in Maya

    Every iteration moves each vertex's weights toward the average of its
    neighbors' weights. That spreads each joint to more vertices, so afterwards
    each vertex only keeps its strongest influences

    Parameters
    ----------
    weights : np.ndarray
        The (N, J) skin weights
    faces : [[int, ...], ...]
        The mesh faces
    iterations : int, optional
        How many times to smooth. Defaults to 3
    maxInfluences : int, optional
        The max number of joints per vertex. Defaults to 4
    strength : float, optional
        How far each iteration moves toward the neighbors' average, from 0 to 1.
        Defaults to 0.5

    Returns
    -------
    : np.ndarray
        The (N, J) smoothed weights
    """
    weights = np.asarray(weights, dtype=np.float64)
    if iterations <= 0:
        return weights.copy()
    numVerts = len(weights)
    edges = meshEdges(faces)
    adjacency = scipy.sparse.csr_array(
        (
            np.ones(2 * len(edges)),
            (np.concatenate(edges.T[::-1]), np.concatenate(edges.T)),
        ),
        shape=(numVerts, numVerts),
    )
    degree = adjacency.sum(axis=1)
    # Vertices without any neighbors just keep their weights
    lonely = degree == 0
    degree[lonely] = 1.0

    for _ in range(iterations):
        average = (adjacency @ weights) / degree[:, None]
        average[lonely] = weights[lonely]
        weights = (1.0 - strength) * weights + strength * average

    # Only keep the strongest influences
    if weights.shape[1] > maxInfluences:
        weak = np.argpartition(-weights, maxInfluences, axis=1)[:, maxInfluences:]
        np.put_along_axis(weights, weak, 0.0, axis=1)
    weights /= weights.sum(axis=1, keepdims=True)
    return weights


def skinRmse(
    restVerts: np.ndarray,
    targetVerts: np.ndarray,
    weights: np.ndarray,
    skin: np.ndarray,
    ignoreVerts: Sequence[int] | None = None,
) -> float:
    """Get the root mean squared error of linear blend skinning against targets

    Parameters
    ----------
    restVerts : np.ndarray
        The (N, 3) rest positions
    targetVerts : np.ndarray
        The (S, N, 3) absolute target positions
    weights : np.ndarray
        The (N, J) skin weights
    skin : np.ndarray
        The (S, J, 4, 4) skinning matrices
    ignoreVerts : [int, ...], optional
        Vertices to leave out of the error

    Returns
    -------
    : float
        The error
    """
    keep = np.ones(len(restVerts), dtype=bool)
    if ignoreVerts is not None:
        keep[np.asarray(ignoreVerts, dtype=int)] = False
    if not keep.any():
        return 0.0
    # Each row is a vertex's homogeneous rest position times its weight for each
    # joint, so multiplying by the stacked skinning matrices skins it
    numShapes, numJoints = skin.shape[:2]
    rows, cols = np.nonzero(weights[keep])
    restH = toHomogeneous(restVerts[keep])
    skinMat = scipy.sparse.csr_array(
        (
            (weights[keep][rows, cols, None] * restH[rows]).ravel(),
            (np.repeat(rows, 4), (cols[:, None] * 4 + np.arange(4)).ravel()),
        ),
        shape=(keep.sum(), numJoints * 4),
    )
    mats = skin[..., :3].transpose(1, 2, 0, 3).reshape(numJoints * 4, numShapes * 3)
    fit = (skinMat @ mats).reshape(-1, numShapes, 3)
    err = ((fit - targetVerts[:, keep].transpose(1, 0, 2)) ** 2).sum()
    return float(np.sqrt(err / (numShapes * keep.sum())))
