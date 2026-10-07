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
"""Decompose blendshape targets into joint poses, skin weights, and residual shapes

This module is intentionally independent of Simplex, Qt, and any DCC. It only needs
numpy (and py_dem_bones for `solveJointPoses`) so it can be used anywhere.

Conventions
-----------
* All matrices are 4x4 with Maya's row-vector convention: points are row vectors
  that get multiplied on the left (``p' = p @ M``), and the translation is in the
  last row. Matrix products read left to right in application order.
* ``restVerts`` is an (N, 3) array, ``targetVerts`` is an (S, N, 3) array of *absolute*
  target positions, and ``jointMats`` is a (J, 4, 4) array of the joints' rest world
  matrices. All of them must be in the same space.
* Skin weights are returned as an (N, J) array.
* ``parents`` is a list of J ints giving the index of each joint's parent within the
  given joints, or -1 if the joint's parent is not part of the solve.

Pose Parameters
---------------
The blendPose node blends per-joint deltas ``D = localPose @ inv(localRest)``.
It decomposes each delta into scale, shear, axis-angle, and translation, adds the
weighted components together, and rebuilds the matrix (its ``outputLog`` plug).
`poseDeltaParams` returns those components as a (..., J, 4, 3) array whose rows are
``(scale - 1, shear, axisAngle, translation)``. Because the node blends these
parameters linearly, they can be collapsed into corrective deltas exactly like
blendshape deltas can.
"""

from __future__ import annotations

from typing import Callable, Sequence

import numpy as np

ProgressCallback = Callable[[int, int, str], None]


def _progress(cb: ProgressCallback | None, value: int, maximum: int, msg: str) -> None:
    if cb is not None:
        cb(value, maximum, msg)


def toHomogeneous(pts: np.ndarray) -> np.ndarray:
    """Append a 1 to the last axis of an array of 3d points"""
    ones = np.ones(pts.shape[:-1] + (1,), dtype=pts.dtype)
    return np.concatenate((pts, ones), axis=-1)


#########################################################################
####                     blendPose compatible math                    ####
#########################################################################


def _rotToQuat(rot: np.ndarray) -> np.ndarray:
    """Convert column-convention 3x3 rotation matrices to xyzw quaternions
    with a non-negative scalar part (Shepperd's method)
    """
    m = rot
    shape = m.shape[:-2]
    m = m.reshape((-1, 3, 3))
    q = np.empty((len(m), 4))

    m00, m11, m22 = m[:, 0, 0], m[:, 1, 1], m[:, 2, 2]
    tr = m00 + m11 + m22
    choice = np.argmax(np.stack((tr, m00, m11, m22), axis=-1), axis=-1)

    i = choice == 0
    s = np.sqrt(1.0 + tr[i]) * 2
    q[i, 3] = 0.25 * s
    q[i, 0] = (m[i, 2, 1] - m[i, 1, 2]) / s
    q[i, 1] = (m[i, 0, 2] - m[i, 2, 0]) / s
    q[i, 2] = (m[i, 1, 0] - m[i, 0, 1]) / s

    i = choice == 1
    s = np.sqrt(1.0 + m00[i] - m11[i] - m22[i]) * 2
    q[i, 3] = (m[i, 2, 1] - m[i, 1, 2]) / s
    q[i, 0] = 0.25 * s
    q[i, 1] = (m[i, 0, 1] + m[i, 1, 0]) / s
    q[i, 2] = (m[i, 0, 2] + m[i, 2, 0]) / s

    i = choice == 2
    s = np.sqrt(1.0 + m11[i] - m00[i] - m22[i]) * 2
    q[i, 3] = (m[i, 0, 2] - m[i, 2, 0]) / s
    q[i, 0] = (m[i, 0, 1] + m[i, 1, 0]) / s
    q[i, 1] = 0.25 * s
    q[i, 2] = (m[i, 1, 2] + m[i, 2, 1]) / s

    i = choice == 3
    s = np.sqrt(1.0 + m22[i] - m00[i] - m11[i]) * 2
    q[i, 3] = (m[i, 1, 0] - m[i, 0, 1]) / s
    q[i, 0] = (m[i, 0, 2] + m[i, 2, 0]) / s
    q[i, 1] = (m[i, 1, 2] + m[i, 2, 1]) / s
    q[i, 2] = 0.25 * s

    q[q[:, 3] < 0] *= -1
    return q.reshape(shape + (4,))


def logRot(mats: np.ndarray) -> np.ndarray:
    """Get the blendPose axis-angle vectors of the rotation part of row-convention matrices

    This matches ``logm`` in matutils.cpp, which returns the lower triangle of the
    skew-symmetric log matrix as ``(S[1,0], S[2,0], S[2,1])``. It is computed through
    quaternions so it stays stable all the way out to 180 degrees.

    Parameters
    ----------
    mats : np.ndarray
        A (..., 4, 4) or (..., 3, 3) array of pure rotation matrices

    Returns
    -------
    : np.ndarray
        A (..., 3) array of axis-angle vectors
    """
    # The transpose of a row-convention matrix is a column-convention matrix
    q = _rotToQuat(np.swapaxes(mats[..., :3, :3], -1, -2))
    v = q[..., :3]
    vlen = np.linalg.norm(v, axis=-1)
    theta = 2.0 * np.arctan2(vlen, q[..., 3])
    with np.errstate(divide="ignore", invalid="ignore"):
        scale = np.where(vlen > 1e-15, theta / vlen, 2.0)
    rotvec = v * scale[..., None]

    # Convert the standard column-convention rotation vector into matutils' convention
    # For row matrices S = (M - M.T) / 2 = -[r]x, so (S10, S20, S21) = (-r2, r1, -r0)
    return np.stack((-rotvec[..., 2], rotvec[..., 1], -rotvec[..., 0]), axis=-1)


def expRot(aa: np.ndarray) -> np.ndarray:
    """Build row-convention 4x4 rotation matrices from blendPose axis-angle vectors

    This matches ``expm`` in matutils.cpp

    Parameters
    ----------
    aa : np.ndarray
        A (..., 3) array of axis-angle vectors

    Returns
    -------
    : np.ndarray
        A (..., 4, 4) array of rotation matrices
    """
    shape = aa.shape[:-1]
    aa = aa.reshape((-1, 3))
    theta = np.linalg.norm(aa, axis=-1)

    skew = np.zeros((len(aa), 3, 3))
    skew[:, 1, 0] = aa[:, 0]
    skew[:, 2, 0] = aa[:, 1]
    skew[:, 2, 1] = aa[:, 2]
    skew[:, 0, 1] = -aa[:, 0]
    skew[:, 0, 2] = -aa[:, 1]
    skew[:, 1, 2] = -aa[:, 2]

    with np.errstate(divide="ignore", invalid="ignore"):
        st = np.where(theta > 1e-12, np.sin(theta) / theta, 1.0)
        ct = np.where(theta > 1e-12, (1 - np.cos(theta)) / (theta * theta), 0.5)

    ret = np.zeros((len(aa), 4, 4))
    ret[:, :3, :3] = (
        np.eye(3)[None] + st[:, None, None] * skew + ct[:, None, None] * (skew @ skew)
    )
    ret[:, 3, 3] = 1.0
    return ret.reshape(shape + (4, 4))


def decomposeMats(mats: np.ndarray):
    """Decompose row-convention matrices into scale, shear, rotation, and translation

    This is a vectorized port of ``decompose`` in matutils.cpp

    Parameters
    ----------
    mats : np.ndarray
        A (..., 4, 4) array of matrices

    Returns
    -------
    : np.ndarray
        The (..., 3) scales
    : np.ndarray
        The (..., 3) shears
    : np.ndarray
        The (..., 4, 4) rotation matrices
    : np.ndarray
        The (..., 3) translations
    """
    m3 = mats[..., :3, :3]
    tran = mats[..., 3, :3].copy()
    mmt = m3 @ np.swapaxes(m3, -1, -2)

    a = np.sqrt(mmt[..., 0, 0])
    d = mmt[..., 0, 1] / a
    e = mmt[..., 0, 2] / a
    b = np.sqrt(mmt[..., 1, 1] - d * d)
    f = (mmt[..., 1, 2] - d * e) / b
    c = np.sqrt(np.maximum(mmt[..., 2, 2] - e * e - f * f, 0.0))

    scsh = np.zeros(mats.shape[:-2] + (3, 3))
    scsh[..., 0, 0] = a
    scsh[..., 1, 0] = d
    scsh[..., 1, 1] = b
    scsh[..., 2, 0] = e
    scsh[..., 2, 1] = f
    scsh[..., 2, 2] = c

    rot = np.zeros(mats.shape)
    rot[..., :3, :3] = np.linalg.inv(scsh) @ m3
    rot[..., 3, 3] = 1.0

    scale = np.stack((a, b, c), axis=-1)
    shear = np.stack((d / b, e / c, f / c), axis=-1)
    return scale, shear, rot, tran


def composeMats(
    scale: np.ndarray, shear: np.ndarray, rot: np.ndarray, tran: np.ndarray
) -> np.ndarray:
    """Build row-convention matrices from components. The inverse of `decomposeMats`

    This is a vectorized port of ``compose`` in matutils.cpp
    """
    shape = tran.shape[:-1]
    sc = np.zeros(shape + (3, 3))
    sc[..., 0, 0] = scale[..., 0]
    sc[..., 1, 1] = scale[..., 1]
    sc[..., 2, 2] = scale[..., 2]

    sh = np.zeros(shape + (3, 3))
    sh[..., :, :] = np.eye(3)
    sh[..., 1, 0] = shear[..., 0]
    sh[..., 2, 0] = shear[..., 1]
    sh[..., 2, 1] = shear[..., 2]

    ret = np.zeros(shape + (4, 4))
    ret[..., :3, :3] = sc @ sh @ rot[..., :3, :3]
    ret[..., 3, :3] = tran
    ret[..., 3, 3] = 1.0
    return ret


def localMats(worldMats: np.ndarray, parents: Sequence[int]) -> np.ndarray:
    """Convert world matrices to matrices local to their parent joint

    Parameters
    ----------
    worldMats : np.ndarray
        A (..., J, 4, 4) array of world matrices
    parents : [int, ...]
        The parent index for each joint, or -1 for no parent

    Returns
    -------
    : np.ndarray
        A (..., J, 4, 4) array of local matrices
    """
    ret = worldMats.copy()
    invs = np.linalg.inv(worldMats)
    for j, p in enumerate(parents):
        if p >= 0:
            ret[..., j, :, :] = worldMats[..., j, :, :] @ invs[..., p, :, :]
    return ret


def worldMats(localMats: np.ndarray, parents: Sequence[int]) -> np.ndarray:
    """Convert local matrices back to world matrices. The inverse of `localMats`"""
    ret = localMats.copy()
    done = [p < 0 for p in parents]
    while not all(done):
        progressed = False
        for j, p in enumerate(parents):
            if done[j] or not done[p]:
                continue
            ret[..., j, :, :] = localMats[..., j, :, :] @ ret[..., p, :, :]
            done[j] = True
            progressed = True
        if not progressed:
            raise ValueError("The joint parents contain a cycle")
    return ret


def poseDeltaParams(restLocal: np.ndarray, poseLocal: np.ndarray) -> np.ndarray:
    """Get the blendPose parameters that move each joint from its rest to a pose

    Parameters
    ----------
    restLocal : np.ndarray
        A (J, 4, 4) array of local rest matrices
    poseLocal : np.ndarray
        A (..., J, 4, 4) array of local pose matrices

    Returns
    -------
    : np.ndarray
        A (..., J, 4, 3) array of ``(scale - 1, shear, axisAngle, translation)``
    """
    deltas = poseLocal @ np.linalg.inv(restLocal)
    scale, shear, rot, tran = decomposeMats(deltas)
    return np.stack((scale - 1.0, shear, logRot(rot), tran), axis=-2)


def paramsToDeltas(params: np.ndarray) -> np.ndarray:
    """Build the delta matrices from blendPose parameters

    Parameters
    ----------
    params : np.ndarray
        A (..., J, 4, 3) array of ``(scale - 1, shear, axisAngle, translation)``

    Returns
    -------
    : np.ndarray
        A (..., J, 4, 4) array of delta matrices
    """
    return composeMats(
        params[..., 0, :] + 1.0,
        params[..., 1, :],
        expRot(params[..., 2, :]),
        params[..., 3, :],
    )


def evaluateBlendPose(
    restLocal: np.ndarray, deltaParams: np.ndarray, weights: np.ndarray
) -> np.ndarray:
    """A numpy implementation of the blendPose node's ``outputLog`` plug

    Parameters
    ----------
    restLocal : np.ndarray
        A (J, 4, 4) array of local rest matrices
    deltaParams : np.ndarray
        A (T, J, 4, 3) array of per-target parameters
    weights : np.ndarray
        A (T,) array of target weights, or an (F, T) array to evaluate many at once

    Returns
    -------
    : np.ndarray
        A (J, 4, 4) or (F, J, 4, 4) array of local pose matrices
    """
    params = np.tensordot(weights, deltaParams, axes=([-1], [0]))
    return paramsToDeltas(params) @ restLocal


def blendPoseTargets(restLocal: np.ndarray, deltaParams: np.ndarray):
    """Get the values to plug into the blendPose node's target attributes

    The targets should have ``targetPoseUseMat`` turned off so that the node uses the
    provided axis-angle values instead of re-deriving them from the matrices.

    Parameters
    ----------
    restLocal : np.ndarray
        A (J, 4, 4) array of local rest matrices (the ``origMat`` values)
    deltaParams : np.ndarray
        A (T, J, 4, 3) array of per-target parameters

    Returns
    -------
    : np.ndarray
        A (T, J, 4, 4) array of ``targetPoseMat`` values
    : np.ndarray
        A (T, J, 3) array of ``targetPoseAA`` values
    """
    return paramsToDeltas(deltaParams) @ restLocal, deltaParams[..., 2, :].copy()


#########################################################################
####                         Skinning helpers                         ####
#########################################################################


def skinMatrices(jointMats: np.ndarray, poseMats: np.ndarray) -> np.ndarray:
    """Get the skinning matrices (bindPreMatrix @ worldMatrix) for each pose

    Parameters
    ----------
    jointMats : np.ndarray
        A (J, 4, 4) array of rest world matrices
    poseMats : np.ndarray
        A (..., J, 4, 4) array of posed world matrices

    Returns
    -------
    : np.ndarray
        A (..., J, 4, 4) array of skinning matrices
    """
    return np.linalg.inv(jointMats) @ poseMats


def skinVerts(
    verts: np.ndarray, weights: np.ndarray, skinMats: np.ndarray
) -> np.ndarray:
    """Linear blend skin points

    Parameters
    ----------
    verts : np.ndarray
        An (N, 3) or (S, N, 3) array of pre-skin points
    weights : np.ndarray
        An (N, J) array of skin weights
    skinMats : np.ndarray
        An (S, J, 4, 4) array of skinning matrices

    Returns
    -------
    : np.ndarray
        An (S, N, 3) array of skinned points
    """
    verts = np.broadcast_to(verts, (len(skinMats),) + verts.shape[-2:])
    out = np.empty(verts.shape)
    for s, mats in enumerate(skinMats):
        jointPts = np.einsum("na,jab->njb", toHomogeneous(verts[s]), mats[..., :3])
        out[s] = np.einsum("nj,njb->nb", weights, jointPts)
    return out


def unskinVerts(
    targetVerts: np.ndarray,
    weights: np.ndarray,
    skinMats: np.ndarray,
    progress: ProgressCallback | None = None,
) -> np.ndarray:
    """Find the pre-skin points that skin to the given targets

    This is the same per-vertex matrix inversion that applyCorrectives does, except that
    linear blend skinning gives us the per-vertex matrices directly

    Parameters
    ----------
    targetVerts : np.ndarray
        An (S, N, 3) array of target points
    weights : np.ndarray
        An (N, J) array of skin weights
    skinMats : np.ndarray
        An (S, J, 4, 4) array of skinning matrices
    progress : callable, optional
        Called as ``progress(value, maximum, message)``

    Returns
    -------
    : np.ndarray
        An (S, N, 3) array of pre-skin points
    """
    out = np.empty(targetVerts.shape)
    for s, mats in enumerate(skinMats):
        _progress(progress, s, len(skinMats), "Un-skinning targets")
        vertMats = np.einsum("nj,jab->nab", weights, mats)
        inv = np.linalg.inv(vertMats)
        out[s] = np.einsum("na,nab->nb", toHomogeneous(targetVerts[s]), inv)[..., :3]
    return out


def nearestJointWeights(restVerts: np.ndarray, jointMats: np.ndarray) -> np.ndarray:
    """Build hard (N, J) weights binding each vertex to its closest joint"""
    jointPos = jointMats[:, 3, :3]
    dists = np.linalg.norm(restVerts[:, None, :] - jointPos[None, :, :], axis=-1)
    weights = np.zeros((len(restVerts), len(jointMats)))
    weights[np.arange(len(restVerts)), np.argmin(dists, axis=1)] = 1.0
    return weights


#########################################################################
####                            Dem Bones                             ####
#########################################################################


def solveJointPoses(
    restVerts: np.ndarray,
    targetVerts: np.ndarray,
    jointMats: np.ndarray,
    faces: Sequence[Sequence[int]],
    weights: np.ndarray | None = None,
    lockJoints: Sequence[int] | None = None,
    lockWeights: bool | np.ndarray = False,
    maxInfluences: int = 4,
    iterations: int = 30,
    transIterations: int = 5,
    weightIterations: int = 3,
    weightsSmooth: float = 1e-4,
    weightsSmoothStep: float = 1.0,
):
    """Use Dem Bones to solve for joint poses and skin weights that match the targets

    The joint rest positions don't change how well the skinning can fit the targets,
    but they are the pivots that the blendPose node rotates around, and they're used
    to build the initial weights if none are given.

    Parameters
    ----------
    restVerts : np.ndarray
        The (N, 3) rest positions
    targetVerts : np.ndarray
        The (S, N, 3) absolute target positions
    jointMats : np.ndarray
        The (J, 4, 4) rest world matrices of the joints
    faces : [[int, ...], ...]
        The mesh faces. Used to smooth the weights
    weights : np.ndarray, optional
        (N, J) initial skin weights. Defaults to binding each vertex to its closest joint
    lockJoints : [int, ...], optional
        Indices of joints that are not allowed to move (like a head joint)
    lockWeights : bool or np.ndarray, optional
        True to keep the given weights. Or an (N,) array between 0 and 1
        saying how much to keep each vertex's weights. Defaults to False
    maxInfluences : int, optional
        The max number of joints per vertex. Defaults to 4
    iterations : int, optional
        The number of global iterations. Defaults to 30
    transIterations : int, optional
        The number of transformation iterations per global iteration. Defaults to 5
    weightIterations : int, optional
        The number of weight iterations per global iteration. Defaults to 3
    weightsSmooth : float, optional
        The weight smoothness regularization. Defaults to 1e-4
    weightsSmoothStep : float, optional
        The step size of the weight smoothing. Defaults to 1.0

    Returns
    -------
    : np.ndarray
        The (S, J, 4, 4) world matrices of each joint for each target
    : np.ndarray
        The (N, J) skin weights
    : float
        The root mean squared error of the fit
    """
    from py_dem_bones import DemBones

    restVerts = np.asarray(restVerts, dtype=np.float64)
    targetVerts = np.asarray(targetVerts, dtype=np.float64)
    jointMats = np.asarray(jointMats, dtype=np.float64)
    numVerts, numFrames, numJoints = len(restVerts), len(targetVerts), len(jointMats)
    if targetVerts.shape[1:] != restVerts.shape:
        raise ValueError("The targets must have the same shape as the rest verts")

    if weights is None:
        weights = nearestJointWeights(restVerts, jointMats)
    weights = np.asarray(weights, dtype=np.float64)
    if weights.shape != (numVerts, numJoints):
        raise ValueError(f"Weights must have shape {(numVerts, numJoints)}")

    db = DemBones()
    db.nV = numVerts
    db.nF = numFrames
    db.nB = numJoints
    db.nS = 1
    db.nnz = min(maxInfluences, numJoints)
    db.nIters = iterations
    db.nTransIters = transIterations
    db.nWeightsIters = weightIterations
    db.weightsSmooth = weightsSmooth
    db.weightsSmoothStep = weightsSmoothStep
    # The affine regularization pulls translations toward the bone's centroid
    # which doesn't make sense when we're giving it real joints. And it keeps
    # the solve from converging even when there is an exact solution
    db.transAffine = 0.0
    db.fStart = np.array([0, numFrames], dtype=np.int32)
    db.subjectID = np.zeros(numFrames, dtype=np.int32)
    db.u = np.ascontiguousarray(restVerts.T)
    db.v = np.ascontiguousarray(
        targetVerts.transpose(0, 2, 1).reshape(3 * numFrames, numVerts)
    )
    db.fv = [list(map(int, f)) for f in faces]
    db.set_weights(np.ascontiguousarray(weights.T))

    # Dem bones only keeps lockM if it's also given the transformations
    db.m = np.tile(np.eye(4), (numFrames, numJoints))
    lockM = np.zeros(numJoints, dtype=np.int32)
    if lockJoints:
        lockM[list(lockJoints)] = 1
    db.lockM = lockM

    if lockWeights is True:
        db.lockW = np.ones(numVerts)
    elif lockWeights is not False:
        db.lockW = np.asarray(lockWeights, dtype=np.float64)

    db.compute()

    # Dem bones uses column-convention skinning matrices: v = sum(w * M @ u)
    # So transpose them to get the row-convention skinning matrices
    colMats = np.asarray(db.m).reshape(numFrames, 4, numJoints, 4).transpose(0, 2, 1, 3)
    rowSkinMats = np.swapaxes(colMats, -1, -2)
    poseMats = jointMats[None] @ rowSkinMats

    # The weights are only approximately normalized, and the DCC will force them to be
    # So normalize them here so everything downstream uses the same weights
    outWeights = np.asarray(db.get_weights()).T.copy()
    outWeights /= outWeights.sum(axis=1, keepdims=True)
    return poseMats, outWeights, db.rmse()


def decomposeTargets(
    restVerts: np.ndarray,
    targetVerts: np.ndarray,
    jointMats: np.ndarray,
    faces: Sequence[Sequence[int]],
    parents: Sequence[int] | None = None,
    progress: ProgressCallback | None = None,
    **solveKwargs,
):
    """Turn a set of blendshape targets into joint poses, skin weights, and residual shapes

    This is the generic version of the whole process for a set of independent targets.
    Each target gets its own pose, and the residual shapes are what has to be applied
    pre-skin so that skinning them in that pose gives back the original target.

    Parameters
    ----------
    restVerts : np.ndarray
        The (N, 3) rest positions
    targetVerts : np.ndarray
        The (S, N, 3) absolute target positions
    jointMats : np.ndarray
        The (J, 4, 4) rest world matrices of the joints
    faces : [[int, ...], ...]
        The mesh faces
    parents : [int, ...], optional
        The parent index of each joint, or -1. Defaults to no parents
    progress : callable, optional
        Called as ``progress(value, maximum, message)``
    **solveKwargs
        Passed to `solveJointPoses`

    Returns
    -------
    : np.ndarray
        The (S, J, 4, 3) blendPose parameters for each target
    : np.ndarray
        The (N, J) skin weights
    : np.ndarray
        The (S, N, 3) pre-skin residual shapes in absolute positions
    : np.ndarray
        The (S, J, 4, 4) world matrices of each joint for each target
    """
    if parents is None:
        parents = [-1] * len(jointMats)
    _progress(progress, 0, 1, "Solving joint poses")
    poseMats, weights, _ = solveJointPoses(
        restVerts, targetVerts, jointMats, faces, **solveKwargs
    )
    params = poseDeltaParams(
        localMats(jointMats, parents), localMats(poseMats, parents)
    )
    residual = unskinVerts(
        targetVerts, weights, skinMatrices(jointMats, poseMats), progress=progress
    )
    return params, weights, residual, poseMats
