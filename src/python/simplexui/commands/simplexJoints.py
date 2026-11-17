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
"""Convert a simplex system of blendshapes into joint poses plus residual blendshapes

The joint poses are stored as blendPose parameters (see jointPoses.py), and because
the blendPose node blends those parameters linearly, they get collapsed into
corrective deltas the same way the shapes do. So when the simplex solver activates
a combo, the joints get the combo's corrective pose on top of its sliders' poses.

The residual shapes are what's left over after the joints are skinned. They are
applied pre-skin, so the full sculpted shapes are un-skinned before being collapsed
back into deltas.

Traversal shapes are left alone. They keep their original deltas, and get no joint motion
"""

from __future__ import annotations

import datetime
import inspect
import itertools
import os
from dataclasses import dataclass, field, replace

import numpy as np
from pysimplex import PySimplex

from ..items import Simplex
from . import jointPosesRigid
from .alembicCommon import readSmpx, unflattenFaces
from .applyCorrectives import buildFullShapes, collapseFullShapes, writeSimplex
from .jointPoses import (
    SolveJob,
    evaluateBlendPose,
    localMats,
    nearestJointWeights,
    paramsToDeltas,
    poseDeltaParams,
    skinMatrices,
    solveJointPoses,
    solveLayeredPoses,
    solveSkinWeights,
    unskinVerts,
    unwrapAxisAngle,
    worldMats,
)
from .jointWeights import skinRmse, smoothWeights, solveSmoothWeights


@dataclass
class SimplexJointData:
    """The joint data that goes along with a simplex system

    Attributes
    ----------
    jointNames : [str, ...]
        The names of the joints
    jointMats : np.ndarray
        The (J, 4, 4) rest world matrices
    parents : [int, ...]
        The parent index of each joint, or -1
    lockJoints : [int, ...]
        The indices of the joints that aren't driven by the system
    weights : np.ndarray
        The (N, J) skin weights
    deltaParams : np.ndarray
        The (numShapes, J, 4, 3) blendPose delta parameters per simplex shape index
    rmse : float
        The root mean squared error of the joint fit
    """

    jointNames: list[str]
    jointMats: np.ndarray
    parents: list[int]
    lockJoints: list[int]
    weights: np.ndarray
    deltaParams: np.ndarray
    rmse: float = 0.0
    shapeNames: list[str] = field(default_factory=list)

    @property
    def drivenJoints(self) -> list[int]:
        """The indices of the joints driven by the blendPose node"""
        locked = set(self.lockJoints)
        return [i for i in range(len(self.jointNames)) if i not in locked]

    @property
    def restLocal(self) -> np.ndarray:
        return localMats(self.jointMats, self.parents)

    def evaluate(self, shapeValues: np.ndarray) -> np.ndarray:
        """Get the world matrices of the joints given the simplex shape values

        Parameters
        ----------
        shapeValues : np.ndarray
            An (S,) array of shape values from the solver,
            or an (F, S) array to evaluate many at once

        Returns
        -------
        : np.ndarray
            A (J, 4, 4) or (F, J, 4, 4) array of world matrices
        """
        local = evaluateBlendPose(self.restLocal, self.deltaParams, shapeValues)
        return worldMats(local, self.parents)

    def save(self, path: str) -> None:
        """Save the joint data to a .npz file"""
        np.savez(
            path,
            jointNames=np.array(self.jointNames),
            jointMats=self.jointMats,
            parents=np.array(self.parents, dtype=int),
            lockJoints=np.array(self.lockJoints, dtype=int),
            weights=self.weights,
            deltaParams=self.deltaParams,
            rmse=np.array(self.rmse),
            shapeNames=np.array(self.shapeNames),
        )

    @classmethod
    def load(cls, path: str) -> SimplexJointData:
        """Load the joint data from a .npz file"""
        with np.load(path) as data:
            return cls(
                jointNames=[str(i) for i in data["jointNames"]],
                jointMats=data["jointMats"],
                parents=[int(i) for i in data["parents"]],
                lockJoints=[int(i) for i in data["lockJoints"]],
                weights=data["weights"],
                deltaParams=data["deltaParams"],
                rmse=float(data["rmse"]),
                shapeNames=[str(i) for i in data["shapeNames"]],
            )


def _checkLockedHierarchy(parents: list[int], lockJoints: list[int]) -> None:
    """Locked joints can't move, so they can't be children of driven joints"""
    locked = set(lockJoints)
    for j in locked:
        p = parents[j]
        while p >= 0:
            if p not in locked:
                raise ValueError(
                    f"Locked joint {j} is a descendant of driven joint {p}. "
                    "Lock its ancestors too, or unlock it"
                )
            p = parents[p]


def jointShapes(simplex: Simplex) -> list:
    """Get the shapes that will get joint poses.
    These are the non-rest shapes of the sliders and combos
    """
    shapes = []
    for item in itertools.chain(simplex.sliders, simplex.combos):
        for pair in item.prog.pairs:
            if not pair.shape.isRest:
                shapes.append(pair.shape)
    return shapes


def jointifySimplex(
    simplex: Simplex,
    solver: PySimplex,
    deltaShapes: np.ndarray,
    restPts: np.ndarray,
    faces: list[list[int]],
    jointMats: np.ndarray,
    jointNames: list[str] | None = None,
    parents: list[int] | None = None,
    lockJoints: list[int] | None = None,
    pBar=None,
    **solveKwargs,
):
    """Split a simplex system into joint poses, skin weights, and residual shapes

    Parameters
    ----------
    simplex : Simplex
        The simplex system
    solver : PySimplex
        An instantiated solver for the system
    deltaShapes : np.ndarray
        The (numShapes, N, 3) shape deltas in simplex.shapes order
    restPts : np.ndarray
        The (N, 3) rest positions
    faces : [[int, ...], ...]
        The mesh faces
    jointMats : np.ndarray
        The (J, 4, 4) rest world matrices of the joints
    jointNames : [str, ...], optional
        The names of the joints. Defaults to "joint_{index}"
    parents : [int, ...], optional
        The parent index of each joint, or -1. Defaults to no parents
    lockJoints : [int, ...], optional
        The indices of joints that aren't allowed to move, like a head joint
    pBar : QProgressDialog, optional
        An optional progress dialog
    **solveKwargs
        Passed to jointPoses.solveJointPoses

    Returns
    -------
    : np.ndarray
        The (numShapes, N, 3) residual shapes as absolute points, ready to write to a .smpx
    : SimplexJointData
        The joint data
    """
    numJoints = len(jointMats)
    if jointNames is None:
        jointNames = [f"joint_{i}" for i in range(numJoints)]
    if parents is None:
        parents = [-1] * numJoints
    lockJoints = sorted(set(lockJoints or []))
    _checkLockedHierarchy(parents, lockJoints)

    shapes = jointShapes(simplex)
    ptsByShape, vecByShape = buildFullShapes(simplex, shapes, deltaShapes, solver, pBar)
    targets = np.array([restPts + ptsByShape[s] for s in shapes])

    if pBar is not None:
        pBar.setLabelText("Solving Joint Poses")
    print("Solving Joint Poses")
    poseMats, weights, rmse = solveJointPoses(
        restPts, targets, jointMats, faces, lockJoints=lockJoints, **solveKwargs
    )
    print(f"Joint fit RMSE: {rmse}")

    return _finishJointify(
        simplex,
        deltaShapes,
        restPts,
        jointMats,
        jointNames,
        parents,
        lockJoints,
        shapes,
        targets,
        vecByShape,
        poseMats,
        weights,
        rmse,
        pBar,
    )


def _finishJointify(
    simplex: Simplex,
    deltaShapes: np.ndarray,
    restPts: np.ndarray,
    jointMats: np.ndarray,
    jointNames: list[str],
    parents: list[int],
    lockJoints: list[int],
    shapes: list,
    targets: np.ndarray,
    vecByShape: dict,
    poseMats: np.ndarray,
    weights: np.ndarray,
    rmse: float,
    pBar=None,
    fullParams: np.ndarray | None = None,
):
    """Collapse the solved poses into blendPose deltas, and build the residual shapes"""
    numJoints = len(jointMats)

    # Collapse the blendPose parameters into deltas. The parameters are (J, 4, 3)
    # per shape, so reshape them to look like (4J, 3) points
    if fullParams is None:
        restLocal = localMats(jointMats, parents)
        fullParams = poseDeltaParams(restLocal, localMats(poseMats, parents))
    paramsByShape = {s: p.reshape((-1, 3)) for s, p in zip(shapes, fullParams)}
    noParams = np.zeros((len(simplex.shapes), numJoints * 4, 3))
    deltaParams = collapseFullShapes(simplex, noParams, paramsByShape, vecByShape, pBar)
    deltaParams = deltaParams.reshape((len(simplex.shapes), numJoints, 4, 3))

    jointData = SimplexJointData(
        jointNames=list(jointNames),
        jointMats=np.asarray(jointMats, dtype=np.float64),
        parents=list(parents),
        lockJoints=lockJoints,
        weights=weights,
        deltaParams=deltaParams,
        rmse=rmse,
        shapeNames=[s.name for s in simplex.shapes],
    )
    newShapePts = _residualShapes(
        simplex, deltaShapes, restPts, shapes, targets, vecByShape, jointData, pBar
    )
    return newShapePts, jointData


def _residualShapes(
    simplex: Simplex,
    deltaShapes: np.ndarray,
    restPts: np.ndarray,
    shapes: list,
    targets: np.ndarray,
    vecByShape: dict,
    jointData: SimplexJointData,
    pBar=None,
) -> np.ndarray:
    """Un-skin the targets with the joint data, and collapse them into residual shapes"""
    # Un-skin using the poses the rig will actually produce. At the exact shape
    # activations these match the solved poses, but this guarantees the residuals
    # are computed against the same thing the blendPose node outputs
    rigPoses = jointData.evaluate(np.array([vecByShape[s] for s in shapes]))
    if pBar is not None:
        pBar.setLabelText("Un-skinning Shapes")
    print("Un-skinning Shapes")
    skin = skinMatrices(jointData.jointMats, rigPoses)
    preSkin = unskinVerts(targets, jointData.weights, skin)

    newPtsByShape = {s: p - restPts for s, p in zip(shapes, preSkin)}
    newShapePts = collapseFullShapes(
        simplex, deltaShapes, newPtsByShape, vecByShape, pBar
    )
    return newShapePts + restPts[None, ...]


def rebuildResiduals(
    inPath: str,
    outPath: str,
    jointData: SimplexJointData,
    jointDataPath: str | None = None,
    pBar=None,
) -> SimplexJointData:
    """Rebuild the residual shapes for joint data whose weights have been edited

    The joint poses are kept as they are, only the residuals change

    Parameters
    ----------
    inPath : str
        The original .smpx file with the full sculpted shapes, NOT the residuals
    outPath : str
        The output .smpx file containing the new residual shapes
    jointData : SimplexJointData
        The joint data with the edited weights
    jointDataPath : str, optional
        Where to write the joint data. Defaults to the outPath with a .npz extension
    pBar : QProgressDialog, optional
        An optional progress dialog

    Returns
    -------
    : SimplexJointData
        A copy of the joint data with the joints-only fit error updated
    """
    if not os.path.isfile(str(inPath)):
        raise OSError("File does not exist: " + str(inPath))

    jsString, _, verts, _, _, _ = readSmpx(inPath)
    simplex = Simplex.buildSystemFromJsonString(jsString, None, forceDummy=True)
    solver = PySimplex(jsString)
    if [s.name for s in simplex.shapes] != jointData.shapeNames:
        raise ValueError(f"The shapes in {inPath} don't match the joint data")
    if jointData.weights.shape != (verts.shape[1], len(jointData.jointNames)):
        raise ValueError(
            f"Expected ({verts.shape[1]}, {len(jointData.jointNames)}) weights, "
            f"got {jointData.weights.shape}"
        )

    restPts = verts[simplex.shapes.index(simplex.restShape)]
    deltaShapes = verts - restPts[None, ...]
    shapes = jointShapes(simplex)
    ptsByShape, vecByShape = buildFullShapes(simplex, shapes, deltaShapes, solver, pBar)
    targets = np.array([restPts + ptsByShape[s] for s in shapes])

    rigPoses = jointData.evaluate(np.array([vecByShape[s] for s in shapes]))
    rmse = _fitRmse(restPts, targets, jointData.weights, jointData.jointMats, rigPoses)
    print(f"Joint fit RMSE: {rmse}")
    jointData = replace(jointData, rmse=rmse)

    newShapePts = _residualShapes(
        simplex, deltaShapes, restPts, shapes, targets, vecByShape, jointData, pBar
    )
    writeSimplex(inPath, outPath, newShapePts, name=simplex.name, pBar=pBar)
    if jointDataPath is None:
        jointDataPath = os.path.splitext(outPath)[0] + ".npz"
    jointData.save(jointDataPath)
    return jointData


def _shapeDepths(simplex: Simplex) -> dict:
    """Get the combo depth of each shape. Slider shapes are 0"""
    depths = {}
    for slider in simplex.sliders:
        for pair in slider.prog.pairs:
            depths[pair.shape] = 0
    for combo in simplex.combos:
        for pair in combo.prog.pairs:
            depths[pair.shape] = len(combo.pairs)
    return depths


# The layered pose solvers, by their poseSolver setting
_POSE_SOLVERS = {
    "demBones": solveLayeredPoses,
    "rigid": jointPosesRigid.solveLayeredPoses,
}

# The weight solvers, by their weightSolver setting
_WEIGHT_SOLVERS = {
    "demBones": solveSkinWeights,
    "smooth": solveSmoothWeights,
}

# The arguments that get filled in by the jointify itself, not by the settings
_POSE_ARGS = {
    "restVerts",
    "targetVerts",
    "jointMats",
    "parents",
    "weights",
    "jobs",
    "shapeJobs",
    "initLocal",
    "lockJoints",
    "ignoreVerts",
    "progress",
}
_WEIGHT_ARGS = {
    "restVerts",
    "targetVerts",
    "jointMats",
    "poseMats",
    "faces",
    "weights",
    "joints",
    "lockWeights",
    "ignoreVerts",
}


def _settingNames(func, filled: set[str]) -> set[str]:
    """Get the names of the settings a solver takes"""
    return set(inspect.signature(func).parameters) - filled


def _splitSettings(
    settings: dict, poseSolver: str, weightSolver: str
) -> tuple[dict, dict]:
    """Split the settings between the pose solver and the weight solver

    Raises
    ------
    ValueError
        If a solver name is unknown, or a setting doesn't belong to either solver
    """
    if poseSolver not in _POSE_SOLVERS:
        raise ValueError(
            f"Unknown poseSolver {poseSolver!r}. Use one of {sorted(_POSE_SOLVERS)}"
        )
    if weightSolver not in _WEIGHT_SOLVERS:
        raise ValueError(
            f"Unknown weightSolver {weightSolver!r}. "
            f"Use one of {sorted(_WEIGHT_SOLVERS)}"
        )
    poseNames = _settingNames(_POSE_SOLVERS[poseSolver], _POSE_ARGS)
    weightNames = _settingNames(_WEIGHT_SOLVERS[weightSolver], _WEIGHT_ARGS)
    unknown = sorted(set(settings) - poseNames - weightNames)
    if unknown:
        raise ValueError(
            f"Unknown settings {unknown}. The {poseSolver!r} pose solver takes "
            f"{sorted(poseNames)}, and the {weightSolver!r} weight solver takes "
            f"{sorted(weightNames)}"
        )
    pose = {k: v for k, v in settings.items() if k in poseNames}
    weight = {k: v for k, v in settings.items() if k in weightNames}
    return pose, weight


def solveSimplexPoses(
    simplex: Simplex,
    shapes: list,
    targets: np.ndarray,
    vecByShape: dict,
    restPts: np.ndarray,
    jointMats: np.ndarray,
    parents: list[int],
    weights: np.ndarray,
    jobs: list[SolveJob],
    shapeJobs: list[list[int]],
    lockJoints: list[int],
    poseSolver: str = "demBones",
    **solveKwargs,
) -> tuple[np.ndarray, np.ndarray]:
    """Solve the layered joint poses for the shapes of a simplex system

    The shapes are solved in order of their combo depth. Each combo starts from the
    pose that the rig would give it without any correction of its own, and its
    rotations are regularized toward that pose. That keeps the combo corrections small

    Parameters
    ----------
    poseSolver : str, optional
        Which layered solve to use. Either "demBones" for
        `jointPoses.solveLayeredPoses`, or "rigid" for
        `jointPosesRigid.solveLayeredPoses`. Defaults to "demBones"
    **solveKwargs
        Passed to the chosen solve

    Returns
    -------
    : np.ndarray
        The (len(shapes), J, 4, 4) world matrices of the joints for each shape
    : np.ndarray
        The (len(shapes), J, 4, 3) blendPose parameters for each shape. Their
        rotations are unwrapped to be as close as possible to the rotations the
        rig would give each shape without any correction of its own
    """
    if poseSolver not in _POSE_SOLVERS:
        raise ValueError(
            f"Unknown poseSolver {poseSolver!r}. Use one of {sorted(_POSE_SOLVERS)}"
        )
    solve = _POSE_SOLVERS[poseSolver]

    restLocal = localMats(jointMats, parents)
    indexByShape = {s: i for i, s in enumerate(simplex.shapes)}
    deltaParams = np.zeros((len(simplex.shapes), len(jointMats), 4, 3))
    poseMats = np.empty((len(shapes),) + jointMats.shape)
    fullParams = np.empty((len(shapes), len(jointMats), 4, 3))

    depths = _shapeDepths(simplex)
    byDepth: dict[int, list[int]] = {}
    for i, shape in enumerate(shapes):
        byDepth.setdefault(depths[shape], []).append(i)

    for depth in sorted(byDepth):
        idxs = byDepth[depth]
        now = datetime.datetime.now().time()
        print(f"{now}: Solving {len(idxs)} shapes at depth {depth}")
        vecs = np.array([vecByShape[shapes[i]] for i in idxs])
        base = np.tensordot(vecs, deltaParams, axes=1)
        poses = solve(
            restPts,
            targets[idxs],
            jointMats,
            parents,
            weights,
            jobs,
            [shapeJobs[i] for i in idxs],
            initLocal=paramsToDeltas(base) @ restLocal,
            lockJoints=lockJoints,
            **solveKwargs,
        )
        poseMats[idxs] = poses
        full = poseDeltaParams(restLocal, localMats(poses, parents))
        full[..., 2, :] = unwrapAxisAngle(full[..., 2, :], base[..., 2, :])
        fullParams[idxs] = full
        for i, f, b in zip(idxs, full, base):
            deltaParams[indexByShape[shapes[i]]] = f - b
    return poseMats, fullParams


def _fitRmse(
    restPts: np.ndarray,
    targets: np.ndarray,
    weights: np.ndarray,
    jointMats: np.ndarray,
    poseMats: np.ndarray,
    ignoreVerts: list[int] | None = None,
) -> float:
    """Get the root mean squared error of the skinned poses against the targets"""
    skin = skinMatrices(jointMats, poseMats)
    return skinRmse(restPts, targets, weights, skin, ignoreVerts)


def jointifySimplexLayered(
    simplex: Simplex,
    solver: PySimplex,
    deltaShapes: np.ndarray,
    restPts: np.ndarray,
    faces: list[list[int]],
    jointMats: np.ndarray,
    jobs: list[SolveJob],
    shapeJobs=None,
    jointNames: list[str] | None = None,
    parents: list[int] | None = None,
    lockJoints: list[int] | None = None,
    weights: np.ndarray | None = None,
    weightRounds: int = 2,
    poseSolver: str = "demBones",
    weightSolver: str = "demBones",
    weightSmoothIterations: int = 3,
    weightSmoothLimit: bool = False,
    ignoreVerts: list[int] | None = None,
    pBar=None,
    **settings,
):
    """Split a simplex system into joint poses, skin weights, and residual shapes
    using a layered solve with fixed skin weights. See `jointPoses.solveLayeredPoses`

    The weights and poses are solved in alternating rounds. First the poses are
    solved with the current weights, then new weights are solved for those poses,
    and repeat. The final round is always a pose solve.

    Parameters
    ----------
    simplex : Simplex
        The simplex system
    solver : PySimplex
        An instantiated solver for the system
    deltaShapes : np.ndarray
        The (numShapes, N, 3) shape deltas in simplex.shapes order
    restPts : np.ndarray
        The (N, 3) rest positions
    faces : [[int, ...], ...]
        The mesh faces
    jointMats : np.ndarray
        The (J, 4, 4) rest world matrices of the joints
    jobs : [SolveJob, ...]
        The solve jobs
    shapeJobs : callable or [[int, ...], ...], optional
        The indices of the jobs to run for each shape in `jointShapes` order, or a
        function that takes the simplex and those shapes and returns them.
        Defaults to running every job on every shape
    jointNames : [str, ...], optional
        The names of the joints. Defaults to "joint_{index}"
    parents : [int, ...], optional
        The parent index of each joint, or -1. Defaults to no parents
    lockJoints : [int, ...], optional
        The indices of joints that aren't allowed to move, like a head joint
    weights : np.ndarray, optional
        The (N, J) starting skin weights. Defaults to binding each vertex to its
        closest joint that is either locked or solved by one of the jobs
    weightRounds : int, optional
        The number of times to solve for new weights. Use 0 to keep the given
        weights. Defaults to 2
    poseSolver : str, optional
        How to solve the poses. Either "demBones" for
        `jointPoses.solveLayeredPoses`, or "rigid" for
        `jointPosesRigid.solveLayeredPoses`. Defaults to "demBones"
    weightSolver : str, optional
        How to solve the weights. Either "demBones" for
        `jointPoses.solveSkinWeights`, or "smooth" for
        `jointWeights.solveSmoothWeights`. Defaults to "demBones"
    weightSmoothIterations : int, optional
        How many times to smooth the final weights before building the residual
        shapes, like flood smoothing them in Maya. Use 0 to skip it. Defaults to 3.
        See `jointWeights.smoothWeights`
    weightSmoothLimit : bool, optional
        Whether the smoothed weights keep the same max number of influences per
        vertex as the weight solve. Smoothing spreads the joints out, so without the
        limit vertices can end up with more influences. Defaults to False
    ignoreVerts : [int, ...], optional
        Vertices that the shapes don't apply to, like the inside of the mouth. They
        aren't used to fit the poses. The smooth weight solve fills in their weights
        from the vertices around them, and the Dem Bones weight solve keeps their
        weights as they are
    pBar : QProgressDialog, optional
        An optional progress dialog
    **settings
        The settings of the solvers, like ``iterations`` for the pose solver, or
        ``smoothness`` and ``maxInfluences`` for the smooth weight solver. Each one
        goes to whichever of the chosen solvers takes it. A setting that neither
        of them takes is an error

    Returns
    -------
    : np.ndarray
        The (numShapes, N, 3) residual shapes as absolute points, ready to write to a .smpx
    : SimplexJointData
        The joint data
    """
    numJoints = len(jointMats)
    if jointNames is None:
        jointNames = [f"joint_{i}" for i in range(numJoints)]
    if parents is None:
        parents = [-1] * numJoints
    lockJoints = sorted(set(lockJoints or []))
    _checkLockedHierarchy(parents, lockJoints)

    shapes = jointShapes(simplex)
    ptsByShape, vecByShape = buildFullShapes(simplex, shapes, deltaShapes, solver, pBar)
    targets = np.array([restPts + ptsByShape[s] for s in shapes])

    if shapeJobs is None:
        shapeJobs = [list(range(len(jobs)))] * len(shapes)
    elif callable(shapeJobs):
        shapeJobs = shapeJobs(simplex, shapes)

    # Check the settings before doing any of the work
    poseSettings, weightSettings = _splitSettings(settings, poseSolver, weightSolver)
    poseSettings["poseSolver"] = poseSolver

    solved = {j for job in jobs for p in job.passes for j in p.joints}
    weightJoints = sorted(solved | set(lockJoints))
    if weights is None:
        weights = nearestJointWeights(restPts, jointMats, weightJoints)
    if ignoreVerts is not None:
        ignoreVerts = sorted({int(i) for i in ignoreVerts})
        poseSettings["ignoreVerts"] = ignoreVerts

    if pBar is not None:
        pBar.setLabelText("Solving Joint Poses")
    for weightRound in range(weightRounds + 1):
        now = datetime.datetime.now().time()
        print(
            f"{now}: Solving Joint Poses (round {weightRound + 1} of {weightRounds + 1})"
        )
        poseMats, fullParams = solveSimplexPoses(
            simplex,
            shapes,
            targets,
            vecByShape,
            restPts,
            jointMats,
            parents,
            weights,
            jobs,
            shapeJobs,
            lockJoints,
            **poseSettings,
        )
        rmse = _fitRmse(restPts, targets, weights, jointMats, poseMats, ignoreVerts)
        now = datetime.datetime.now().time()
        print(f"{now}: Joint fit RMSE: {rmse}")
        if weightRound == weightRounds:
            break
        print("Solving Skin Weights")
        if weightSolver == "smooth":
            weights, weightRmse = solveSmoothWeights(
                restPts,
                targets,
                jointMats,
                poseMats,
                faces,
                joints=weightJoints,
                ignoreVerts=ignoreVerts,
                **weightSettings,
            )
        else:
            # Dem Bones can't leave vertices out of the fit, so keep their weights
            kwargs = dict(weightSettings)
            if ignoreVerts:
                lock = np.zeros(len(restPts))
                lock[ignoreVerts] = 1.0
                kwargs["lockWeights"] = lock
            weights, weightRmse = solveSkinWeights(
                restPts, targets, jointMats, poseMats, faces, weights, **kwargs
            )
        now = datetime.datetime.now().time()
        print(f"{now}: Weight fit RMSE: {weightRmse}")

    if weightSmoothIterations > 0:
        maxInfluences = None
        if weightSmoothLimit:
            maxInfluences = weightSettings.get(
                "maxInfluences",
                inspect.signature(_WEIGHT_SOLVERS[weightSolver])
                .parameters["maxInfluences"]
                .default,
            )
        print(f"Smoothing the weights {weightSmoothIterations} times")
        weights = smoothWeights(weights, faces, weightSmoothIterations, maxInfluences)
        rmse = _fitRmse(restPts, targets, weights, jointMats, poseMats, ignoreVerts)
        now = datetime.datetime.now().time()
        print(f"{now}: Joint fit RMSE with the smoothed weights: {rmse}")

    return _finishJointify(
        simplex,
        deltaShapes,
        restPts,
        jointMats,
        jointNames,
        parents,
        lockJoints,
        shapes,
        targets,
        vecByShape,
        poseMats,
        weights,
        rmse,
        pBar,
        fullParams=fullParams,
    )


def jointifySmpx(
    inPath: str,
    outPath: str,
    jointMats: np.ndarray,
    jointNames: list[str] | None = None,
    parents: list[int] | None = None,
    lockJoints: list[int] | None = None,
    jointDataPath: str | None = None,
    jobs: list[SolveJob] | None = None,
    pBar=None,
    **solveKwargs,
) -> SimplexJointData:
    """Read a .smpx file, split it into joints and residual shapes, and write the results

    Parameters
    ----------
    inPath : str
        The input .smpx file
    outPath : str
        The output .smpx file containing the residual shapes
    jointMats : np.ndarray
        The (J, 4, 4) rest world matrices of the joints
    jointNames : [str, ...], optional
        The names of the joints
    parents : [int, ...], optional
        The parent index of each joint, or -1. Defaults to no parents
    lockJoints : [int, ...], optional
        The indices of joints that aren't allowed to move, like a head joint
    jointDataPath : str, optional
        Where to write the joint data. Defaults to the outPath with a .npz extension
    jobs : [SolveJob, ...], optional
        If given, do a layered solve with these jobs. See `jointifySimplexLayered`
    pBar : QProgressDialog, optional
        An optional progress dialog
    **solveKwargs
        Passed to `jointifySimplexLayered` if jobs are given,
        otherwise to jointPoses.solveJointPoses

    Returns
    -------
    : SimplexJointData
        The joint data
    """
    if not os.path.isfile(str(inPath)):
        raise OSError("File does not exist: " + str(inPath))

    jsString, counts, verts, flatFaces, _, _ = readSmpx(inPath)
    simplex = Simplex.buildSystemFromJsonString(jsString, None, forceDummy=True)
    solver = PySimplex(jsString)

    restIdx = simplex.shapes.index(simplex.restShape)
    restPts = verts[restIdx]
    deltaShapes = verts - restPts[None, ...]
    faces = unflattenFaces(flatFaces, counts)

    if jobs is not None:
        solveKwargs["jobs"] = jobs
        jointify = jointifySimplexLayered
    else:
        jointify = jointifySimplex
    newShapePts, jointData = jointify(
        simplex,
        solver,
        deltaShapes,
        restPts,
        faces,
        jointMats,
        jointNames=jointNames,
        parents=parents,
        lockJoints=lockJoints,
        pBar=pBar,
        **solveKwargs,
    )

    writeSimplex(inPath, outPath, newShapePts, name=simplex.name, pBar=pBar)
    if jointDataPath is None:
        jointDataPath = os.path.splitext(outPath)[0] + ".npz"
    jointData.save(jointDataPath)
    return jointData
