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

from dataclasses import dataclass, field
from typing import Callable, Sequence

import numpy as np

ProgressCallback = Callable[[int, int, str], None]


def _defaultProgressCallback(value, maximum, msg):
    perc = 100 * value / maximum
    print(msg, f"{value + 1}/{maximum + 1} : {perc:.2f}%")


def _progress(cb: ProgressCallback | None, value: int, maximum: int, msg: str) -> None:
    if cb is not None:
        cb(value, maximum, msg)
    else:
        _defaultProgressCallback(value, maximum, msg)


def toHomogeneous(pts: np.ndarray) -> np.ndarray:
    """Append a 1 to the last axis of an array of 3d points"""
    ones = np.ones(pts.shape[:-1] + (pts.shape[-1] + 1,), dtype=pts.dtype)
    ones[:, :-1] = pts
    return ones


# np.linalg.inv makes one LAPACK call per matrix, and some OpenBLAS builds
# (including the one numpy ships on windows) spend ~30us of thread synchronization
# on each of those calls. With hundreds of thousands of tiny matrices that's minutes
# of overhead, so these do the inversions witH closed-form elementwise math instead


def inv3(mats: np.ndarray) -> np.ndarray:
    """Invert an array of 3x3 matrices using the adjugate

    Parameters
    ----------
    mats : np.ndarray
        A (..., 3, 3) array of matrices

    Returns
    -------
    : np.ndarray
        A (..., 3, 3) array of the inverse matrices
    """
    m = mats
    adj = np.empty(m.shape)
    adj[..., 0, 0] = m[..., 1, 1] * m[..., 2, 2] - m[..., 1, 2] * m[..., 2, 1]
    adj[..., 0, 1] = m[..., 0, 2] * m[..., 2, 1] - m[..., 0, 1] * m[..., 2, 2]
    adj[..., 0, 2] = m[..., 0, 1] * m[..., 1, 2] - m[..., 0, 2] * m[..., 1, 1]
    adj[..., 1, 0] = m[..., 1, 2] * m[..., 2, 0] - m[..., 1, 0] * m[..., 2, 2]
    adj[..., 1, 1] = m[..., 0, 0] * m[..., 2, 2] - m[..., 0, 2] * m[..., 2, 0]
    adj[..., 1, 2] = m[..., 0, 2] * m[..., 1, 0] - m[..., 0, 0] * m[..., 1, 2]
    adj[..., 2, 0] = m[..., 1, 0] * m[..., 2, 1] - m[..., 1, 1] * m[..., 2, 0]
    adj[..., 2, 1] = m[..., 0, 1] * m[..., 2, 0] - m[..., 0, 0] * m[..., 2, 1]
    adj[..., 2, 2] = m[..., 0, 0] * m[..., 1, 1] - m[..., 0, 1] * m[..., 1, 0]
    det = (
        m[..., 0, 0] * adj[..., 0, 0]
        + m[..., 0, 1] * adj[..., 1, 0]
        + m[..., 0, 2] * adj[..., 2, 0]
    )
    return adj / det[..., None, None]


def invAffine(mats: np.ndarray) -> np.ndarray:
    """Invert an array of row-convention 4x4 affine matrices

    The last column of each matrix must be (0, 0, 0, 1)

    Parameters
    ----------
    mats : np.ndarray
        A (..., 4, 4) array of affine matrices

    Returns
    -------
    : np.ndarray
        A (..., 4, 4) array of the inverse matrices
    """
    rinv = inv3(mats[..., :3, :3])
    ret = np.zeros(mats.shape)
    ret[..., :3, :3] = rinv
    ret[..., 3, :3] = -(mats[..., 3:4, :3] @ rinv)[..., 0, :]
    ret[..., 3, 3] = 1.0
    return ret


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
    rot[..., :3, :3] = inv3(scsh) @ m3
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
    invs = invAffine(worldMats)
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
    deltas = poseLocal @ invAffine(restLocal)
    scale, shear, rot, tran = decomposeMats(deltas)
    return np.stack((scale - 1.0, shear, logRot(rot), tran), axis=-2)


def unwrapAxisAngle(aa: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """Get the axis-angle vectors that are equivalent to ``aa`` but closest to ``ref``

    Rotating by an extra 360 degrees doesn't change a rotation, but it does change
    the blendPose parameters. `logRot` always returns angles up to 180 degrees, so
    when a pose is relative to a big rotation (like the sum of a combo's sliders),
    use this to keep their difference small

    Parameters
    ----------
    aa : np.ndarray
        A (..., 3) array of axis-angle vectors
    ref : np.ndarray
        A (..., 3) array of reference axis-angle vectors

    Returns
    -------
    : np.ndarray
        A (..., 3) array of axis-angle vectors
    """
    theta = np.linalg.norm(aa, axis=-1)
    refLen = np.linalg.norm(ref, axis=-1)
    with np.errstate(divide="ignore", invalid="ignore"):
        axis = np.where(
            (theta > 1.0e-8)[..., None],
            aa / theta[..., None],
            ref / np.where(refLen > 1.0e-8, refLen, 1.0)[..., None],
        )
    proj = (ref * axis).sum(axis=-1)
    turns = np.round((proj - theta) / (2 * np.pi))
    return axis * (theta + 2 * np.pi * turns)[..., None]


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
    return invAffine(jointMats) @ poseMats


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
        vertMats = _blendMats(weights, mats)
        out[s] = np.einsum("na,nab->nb", toHomogeneous(verts[s]), vertMats[..., :3])
    return out


def _blendMats(weights: np.ndarray, mats: np.ndarray) -> np.ndarray:
    """Get the (N, 4, 4) weighted sum of the (J, 4, 4) matrices for each vertex"""
    # A matmul is much faster than the equivalent einsum
    return (weights @ mats.reshape((len(mats), 16))).reshape((len(weights), 4, 4))


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
        # The weights are normalized, so the blended matrices are still affine
        inv = invAffine(_blendMats(weights, mats))
        out[s] = np.einsum("na,nab->nb", toHomogeneous(targetVerts[s]), inv)[..., :3]
    return out


def nearestJointWeights(
    restVerts: np.ndarray,
    jointMats: np.ndarray,
    joints: Sequence[int] | None = None,
) -> np.ndarray:
    """Build hard (N, J) weights binding each vertex to its closest joint

    Parameters
    ----------
    restVerts : np.ndarray
        The (N, 3) rest positions
    jointMats : np.ndarray
        The (J, 4, 4) rest world matrices
    joints : [int, ...], optional
        Only bind to these joints. Defaults to all of them
    """
    if joints is None:
        joints = range(len(jointMats))
    joints = np.asarray(list(joints), dtype=int)
    jointPos = jointMats[joints, 3, :3]
    dists = np.linalg.norm(restVerts[:, None, :] - jointPos[None, :, :], axis=-1)
    weights = np.zeros((len(restVerts), len(jointMats)))
    weights[np.arange(len(restVerts)), joints[np.argmin(dists, axis=1)]] = 1.0
    return weights


#########################################################################
####                        Layered pose solve                        ####
#########################################################################


@dataclass
class SolvePass:
    """A set of joints that get solved together

    Attributes
    ----------
    name : str
        The name of the pass
    joints : {int: (bool, bool)}
        The indices of the joints to solve, mapped to whether each one
        is allowed to (rotate, translate). Dem Bones always solves full rigid
        transforms, so for now these flags are ignored
    rotReg : float
        How strongly the rotations are pulled toward their starting local rotations.
        At 1.0 the regularization is weighted the same as the data, which roughly
        halves the rotations. This gets multiplied by the solve's rotRegScale
    """

    name: str
    joints: dict[int, tuple[bool, bool]]
    rotReg: float = 0.5


@dataclass
class SolveJob:
    """A sequence of passes that get run in order

    Attributes
    ----------
    name : str
        The name of the job
    passes : [SolvePass, ...]
        The passes to run
    """

    name: str
    passes: list[SolvePass] = field(default_factory=list)


def _jointOrder(parents: Sequence[int]) -> list[int]:
    """Get the joint indices sorted so parents come before their children"""
    order = [j for j, p in enumerate(parents) if p < 0]
    children: dict[int, list[int]] = {}
    for j, p in enumerate(parents):
        if p >= 0:
            children.setdefault(p, []).append(j)
    i = 0
    while i < len(order):
        order.extend(children.get(order[i], []))
        i += 1
    if len(order) != len(parents):
        raise ValueError("The joint parents contain a cycle")
    return order


def _solvePass(
    restVerts: np.ndarray,
    targetVerts: np.ndarray,
    jointMats: np.ndarray,
    invRest: np.ndarray,
    parents: Sequence[int],
    order: Sequence[int],
    weights: np.ndarray,
    local: np.ndarray,
    priorLocal: np.ndarray,
    solvePass: SolvePass,
    iterations: int,
    rotRegScale: float,
    minWeight: float,
) -> np.ndarray:
    """Run a single pass of the layered solve on a chunk of shapes with Dem Bones

    Every joint that isn't solved in this pass keeps its local transform, so it
    gets carried along by its closest solved ancestor. Dem Bones can't do that
    directly, so the weights of the carried joints are added to their carrier, and
    each carrier is solved as a single bone. This is exact when the carried joints'
    local transforms are at rest, and approximate otherwise. Joints with no solved
    ancestor keep their world transforms, and are given to Dem Bones as locked bones

    Dem Bones doesn't have any rotation regularization, so it's done with extra
    anchor points. Each free joint gets a few anchors that are only bound to it, and
    their targets are where the joint would put them with its prior local rotation.
    The anchor targets are centered on where the joint currently puts their center,
    so they only pull on the rotation. That center moves as the joint does, so the
    anchors get updated between each Dem Bones iteration

    Returns
    -------
    : np.ndarray
        The updated (S, J, 4, 4) local matrices
    """
    skin = invRest[None] @ worldMats(local, parents)

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
        totals = {j: weights[:, carriedBy[j]].sum(axis=1) for j in free}
        keep = [j for j in free if totals[j].max() >= minWeight]
        if len(keep) == len(free):
            break
        free = keep
    if not free:
        return local

    # Only the points the free joints move affect their solve. The joints that
    # aren't carried, but still influence those points, become locked bones
    freeWeights = np.stack([totals[j] for j in free], axis=1)
    vIdx = np.nonzero(freeWeights.sum(axis=1))[0]
    fixed = [
        k for k in range(len(parents)) if carrier[k] < 0 and weights[vIdx, k].any()
    ]
    bones = free + fixed
    boneWeights = np.concatenate((freeWeights[vIdx], weights[vIdx][:, fixed]), axis=1)
    pts, tgts = restVerts[vIdx], targetVerts[:, vIdx]

    # The carried joints keep their offset from their carrier
    offsets = {
        k: skin[:, k] @ invAffine(skin[:, j])
        for j in free
        for k in carriedBy[j]
        if k != j
    }

    def assemble(solved):
        """Get every joint's (S, J, 4, 4) skin matrices from the solved bones"""
        ret = skin.copy()
        for i, j in enumerate(free):
            ret[:, j] = solved[:, i]
            for k in carriedBy[j]:
                if k != j:
                    ret[:, k] = offsets[k] @ solved[:, i]
        return ret

    # Dem Bones expects every point's weights to sum to 1, so the anchors get a
    # weight of 1 and their spread sets their pull instead. Six anchors per bone
    # (two along each principal axis) match the second moment of its points
    # weighted by rotReg and the squared skin weights, the same way the data
    # points are weighted. Using so few anchors also means their pull on the
    # translation is tiny before they settle, so they settle quickly
    rotReg = solvePass.rotReg * rotRegScale
    numAnchors = 6 * len(free) if rotReg > 0 else 0
    if numAnchors:
        centers = np.empty((len(free), 3))
        anchorOffsets = np.empty((len(free), 6, 3))
        for i in range(len(free)):
            sqW = freeWeights[vIdx, i] ** 2
            centers[i] = sqW @ pts / sqW.sum()
            rel = pts - centers[i]
            moment = rotReg * np.einsum("n,ni,nk->ik", sqW, rel, rel)
            # Keep the regularization in effect when the points are degenerate,
            # like when a joint only influences a line of points
            moment += np.eye(3) * 0.01 * np.trace(moment)
            vals, vecs = np.linalg.eigh(moment)
            axes = (vecs * np.sqrt(np.maximum(vals, 0.0) / 2)).T
            anchorOffsets[i] = np.concatenate((axes, -axes))
        anchorBone = np.repeat(np.arange(len(free)), 6)
        anchorOffsets = anchorOffsets.reshape(-1, 3)
        centersH = toHomogeneous(centers)
        anchorWeights = np.zeros((numAnchors, len(bones)))
        anchorWeights[np.arange(numAnchors), anchorBone] = 1.0
        pts = np.concatenate((pts, centers[anchorBone] + anchorOffsets))
        boneWeights = np.concatenate((boneWeights, anchorWeights))
        tgts = np.concatenate((tgts, np.zeros((len(tgts), numAnchors, 3))), axis=1)

    def anchorTargets(solved):
        """Get the (S, A, 3) anchor targets for the current solved bones"""
        world = jointMats[None] @ assemble(solved)
        priorRot = np.empty((len(local), len(free), 3, 3))
        for i, j in enumerate(free):
            p = parents[j]
            priorWorld = priorLocal[:, j] if p < 0 else priorLocal[:, j] @ world[:, p]
            priorRot[:, i] = (invRest[j] @ priorWorld)[:, :3, :3]
        curCenters = np.einsum("bk,sbkj->sbj", centersH, solved[:, : len(free), :, :3])
        rotated = np.einsum("ak,sakj->saj", anchorOffsets, priorRot[:, anchorBone])
        return rotated + curCenters[:, anchorBone]

    db = _buildDemBones(
        pts,
        tgts,
        [],
        boneWeights,
        len(bones),
        len(bones),
        0,
        0.0,
        1.0,
        True,
    )
    # The weights are fixed, so there's no reason to loop the global iterations
    db.nIters = 1
    db.m = _toDemBonesMats(skin[:, bones])
    db.lockM = np.array([0] * len(free) + [1] * len(fixed), dtype=np.int32)
    if numAnchors:
        db.nTransIters = 1
        solved = skin[:, bones]
        for _ in range(iterations):
            tgts[:, len(vIdx) :] = anchorTargets(solved)
            db.v = _toDemBonesVerts(tgts)
            db.compute()
            solved = _fromDemBonesMats(db.m, len(local), len(bones))
    else:
        db.nTransIters = iterations
        db.compute()
        solved = _fromDemBonesMats(db.m, len(local), len(bones))

    return localMats(jointMats[None] @ assemble(solved), parents)


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
    iterations: int = 10,
    rotRegScale: float = 0.2,
    minWeight: float = 0.3,
    chunkSize: int = 64,
    ignoreVerts: Sequence[int] | None = None,
    progress: ProgressCallback | None = None,
) -> np.ndarray:
    """Solve joint poses in layers with Dem Bones, keeping the skin weights fixed

    This roughly follows how the MetaHuman expression editor fits joints. Each job is
    a sequence of passes, and each pass is a Dem Bones solve of a subset of the
    joints, starting from the result of the previous pass. Joints that aren't solved
    yet get carried along by their solved ancestors, so the early passes solve the
    big parent joints with the influence of their whole hierarchy, and the later
    passes refine their children. See `_solvePass`

    The rotations are regularized toward the starting local rotations. That, plus
    starting each fit from the previous result, keeps the poses consistent so they
    blend well

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
        The number of Dem Bones transformation iterations for each pass. Defaults to 10
    rotRegScale : float, optional
        A multiplier on each pass's rotation regularization. Use 0 to turn it off.
        Defaults to 0.2
    minWeight : float, optional
        Joints whose max weight (including the joints they carry) is below this
        don't get solved, so they just follow their parents. Defaults to 0.3
    chunkSize : int, optional
        How many targets to give Dem Bones at once. Defaults to 64
    ignoreVerts : [int, ...], optional
        Vertices that the targets don't apply to. They aren't used to fit the poses
    progress : callable, optional
        Called as ``progress(value, maximum, message)``

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

    # The passes only fit the vertices with weights, so drop the ignored ones
    if ignoreVerts is not None and len(ignoreVerts):
        weights = weights.copy()
        weights[np.asarray(ignoreVerts, dtype=int)] = 0.0

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

    # Targets with the same jobs get solved together
    groups: dict[tuple[int, ...], list[int]] = {}
    for s, jobIdxs in enumerate(shapeJobs):
        groups.setdefault(tuple(jobIdxs), []).append(s)
    chunks = [
        (jobIdxs, idxs[i : i + chunkSize])
        for jobIdxs, idxs in groups.items()
        for i in range(0, len(idxs), chunkSize)
    ]

    out = np.empty((numShapes,) + jointMats.shape)
    done = 0
    for jobIdxs, idxs in chunks:
        prior = np.array(initLocal[idxs])
        local = prior.copy()
        tgts = targetVerts[idxs]
        for ji in jobIdxs:
            for solvePass in solveJobs[ji].passes:
                _progress(
                    progress, done, numShapes, f"{solveJobs[ji].name}: {solvePass.name}"
                )
                local = _solvePass(
                    restVerts,
                    tgts,
                    jointMats,
                    invRest,
                    parents,
                    order,
                    weights,
                    local,
                    prior,
                    solvePass,
                    iterations,
                    rotRegScale,
                    minWeight,
                )
        out[idxs] = worldMats(local, parents)
        done += len(idxs)
    _progress(progress, numShapes, numShapes, "Done")
    return out


#########################################################################
####                            Dem Bones                            ####
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
    iterations: int = 10,
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

    db = _buildDemBones(
        restVerts,
        targetVerts,
        faces,
        weights,
        numJoints,
        maxInfluences,
        weightIterations,
        weightsSmooth,
        weightsSmoothStep,
        lockWeights,
    )
    db.nIters = iterations
    db.nTransIters = transIterations

    # Dem bones only keeps lockM if it's also given the transformations
    db.m = np.tile(np.eye(4), (numFrames, numJoints))
    lockM = np.zeros(numJoints, dtype=np.int32)
    if lockJoints:
        lockM[list(lockJoints)] = 1
    db.lockM = lockM

    db.compute()

    poseMats = jointMats[None] @ _fromDemBonesMats(db.m, numFrames, numJoints)
    return poseMats, _demBonesWeights(db), db.rmse()


def _buildDemBones(
    restVerts: np.ndarray,
    targetVerts: np.ndarray,
    faces: Sequence[Sequence[int]],
    weights: np.ndarray,
    numJoints: int,
    maxInfluences: int,
    weightIterations: int,
    weightsSmooth: float,
    weightsSmoothStep: float,
    lockWeights: bool | np.ndarray,
):
    """Build a DemBones object with everything but the transformations"""
    from py_dem_bones import DemBones

    numVerts, numFrames = len(restVerts), len(targetVerts)
    db = DemBones()
    db.nV = numVerts
    db.nF = numFrames
    db.nB = numJoints
    db.nS = 1
    db.nnz = min(maxInfluences, numJoints)
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
    db.v = _toDemBonesVerts(targetVerts)
    db.fv = [list(map(int, f)) for f in faces]
    db.set_weights(np.ascontiguousarray(weights.T))

    if lockWeights is True:
        db.lockW = np.ones(numVerts)
    elif lockWeights is not False:
        db.lockW = np.asarray(lockWeights, dtype=np.float64)
    return db


def _demBonesWeights(db) -> np.ndarray:
    """Get the (N, J) weights out of a DemBones object"""
    # The weights are only approximately normalized, and the DCC will force them to be
    # So normalize them here so everything downstream uses the same weights
    outWeights = np.asarray(db.get_weights()).T.copy()
    outWeights /= outWeights.sum(axis=1, keepdims=True)
    return outWeights


# Dem bones uses column-convention skinning matrices: v = sum(w * M @ u)
# stored as a (4 * numFrames, 4 * numBones) block matrix
def _toDemBonesVerts(targetVerts: np.ndarray) -> np.ndarray:
    """Convert (S, N, 3) target positions to Dem Bones' (3 * S, N) layout"""
    numFrames, numVerts = targetVerts.shape[:2]
    return np.ascontiguousarray(
        targetVerts.transpose(0, 2, 1).reshape(3 * numFrames, numVerts)
    )


def _toDemBonesMats(skinMats: np.ndarray) -> np.ndarray:
    """Convert (S, B, 4, 4) row-convention skinning matrices to Dem Bones' layout"""
    numFrames, numBones = skinMats.shape[:2]
    colMats = np.swapaxes(skinMats, -1, -2)
    return np.ascontiguousarray(
        colMats.transpose(0, 2, 1, 3).reshape(4 * numFrames, 4 * numBones)
    )


def _fromDemBonesMats(m, numFrames: int, numBones: int) -> np.ndarray:
    """Convert Dem Bones' matrices to (S, B, 4, 4) row-convention skinning matrices"""
    colMats = np.asarray(m).reshape(numFrames, 4, numBones, 4).transpose(0, 2, 1, 3)
    return np.swapaxes(colMats, -1, -2)


def solveSkinWeights(
    restVerts: np.ndarray,
    targetVerts: np.ndarray,
    jointMats: np.ndarray,
    poseMats: np.ndarray,
    faces: Sequence[Sequence[int]],
    weights: np.ndarray,
    lockWeights: bool | np.ndarray = False,
    maxInfluences: int = 4,
    weightIterations: int = 3,
    weightsSmooth: float = 1e-4,
    weightsSmoothStep: float = 1.0,
):
    """Use Dem Bones to solve for skin weights while keeping the joint poses fixed

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
        The mesh faces. Used to smooth the weights
    weights : np.ndarray
        (N, J) initial skin weights. Each vertex can only pick up influences
        near the ones it already has, so these shouldn't be too sparse
    lockWeights : bool or np.ndarray, optional
        True to keep the given weights. Or an (N,) array between 0 and 1
        saying how much to keep each vertex's weights. Defaults to False
    maxInfluences : int, optional
        The max number of joints per vertex. Defaults to 4
    weightIterations : int, optional
        The number of weight iterations. Defaults to 3
    weightsSmooth : float, optional
        The weight smoothness regularization. Defaults to 1e-4
    weightsSmoothStep : float, optional
        The step size of the weight smoothing. Defaults to 1.0

    Returns
    -------
    : np.ndarray
        The (N, J) skin weights
    : float
        The root mean squared error of the fit
    """
    restVerts = np.asarray(restVerts, dtype=np.float64)
    targetVerts = np.asarray(targetVerts, dtype=np.float64)
    numFrames, numJoints = len(targetVerts), len(jointMats)

    db = _buildDemBones(
        restVerts,
        targetVerts,
        faces,
        np.asarray(weights, dtype=np.float64),
        numJoints,
        maxInfluences,
        weightIterations,
        weightsSmooth,
        weightsSmoothStep,
        lockWeights,
    )
    db.nIters = 1
    db.nTransIters = 0

    # Lock every joint so that only the weights get updated
    db.m = _toDemBonesMats(skinMatrices(jointMats, poseMats))
    db.lockM = np.ones(numJoints, dtype=np.int32)

    db.compute()
    return _demBonesWeights(db), db.rmse()


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
