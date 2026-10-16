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
"""Build a Maya rig where a simplex node drives both a blendShape and a blendPose

Typical usage::

    from simplexui.commands.mayaJointRig import jointifySmpxFromScene, buildJointRig

    joints = cmds.ls("face_*_jnt", type="joint")
    jointData = jointifySmpxFromScene(
        "face.smpx", "face_residual.smpx", ["head_jnt"] + joints, lockJoints=["head_jnt"]
    )
    simplex = buildJointRig("face_residual.smpx", jointData)

The driven joints get their local transforms zeroed, and the blendPose output is
connected to their offsetParentMatrix. That leaves their translate/rotate/scale free
for animators to layer on top of the system.
"""

from __future__ import annotations

import maya.api.OpenMaya as om2
import maya.api.OpenMayaAnim as oma2
import maya.cmds as cmds
import numpy as np

from ..items import Simplex
from .jointPoses import blendPoseTargets
from .simplexJoints import SimplexJointData, jointifySmpx

# Below this, a joint's delta for a shape is considered to be no motion at all
PARAM_EPSILON = 1e-9


def _getMatrix(plug: str) -> np.ndarray:
    return np.array(cmds.getAttr(plug)).reshape((4, 4))


def _setMatrix(plug: str, mat: np.ndarray) -> None:
    cmds.setAttr(plug, *mat.flatten().tolist(), type="matrix")


def getJointMatrices(joints: list[str]) -> np.ndarray:
    """Get the (J, 4, 4) world matrices of the given joints"""
    return np.array([_getMatrix(f"{j}.worldMatrix[0]") for j in joints])


def getJointParents(joints: list[str]) -> list[int]:
    """Get the index of the closest ancestor of each joint that is also in the list, or -1"""
    longNames = cmds.ls(joints, long=True)
    if len(longNames) != len(joints):
        raise ValueError("Some joints are missing, or have non-unique names")
    indexByName = {n: i for i, n in enumerate(longNames)}
    parents = []
    for name in longNames:
        parent = -1
        path = name.rsplit("|", 1)[0]
        while path:
            if path in indexByName:
                parent = indexByName[path]
                break
            path = path.rsplit("|", 1)[0]
        parents.append(parent)
    return parents


def jointifySmpxFromScene(
    inPath: str,
    outPath: str,
    joints: list[str],
    lockJoints: list[str] | None = None,
    jointDataPath: str | None = None,
    pBar=None,
    **solveKwargs,
) -> SimplexJointData:
    """Split a .smpx file into joints and residual shapes using joints from the scene

    The joints must be in their rest pose, and in the same space as the .smpx points

    Parameters
    ----------
    inPath : str
        The input .smpx file
    outPath : str
        The output .smpx file containing the residual shapes
    joints : [str, ...]
        The joints to use
    lockJoints : [str, ...], optional
        Joints that aren't allowed to move, like a head joint. They will still
        be skin influences
    jointDataPath : str, optional
        Where to write the joint data. Defaults to the outPath with a .npz extension
    pBar : QProgressDialog, optional
        An optional progress dialog
    **solveKwargs
        Passed to simplexJoints.jointifySmpx. Use
        jointSolveConfig.solveKwargsFromConfig to get a layered solve

    Returns
    -------
    : SimplexJointData
        The joint data
    """
    lockSet = set(cmds.ls(lockJoints or [], long=True))
    lockIdxs = [i for i, j in enumerate(cmds.ls(joints, long=True)) if j in lockSet]
    return jointifySmpx(
        inPath,
        outPath,
        getJointMatrices(joints),
        jointNames=list(joints),
        parents=getJointParents(joints),
        lockJoints=lockIdxs,
        jointDataPath=jointDataPath,
        pBar=pBar,
        **solveKwargs,
    )


def setupBlendPose(simplex: Simplex, jointData: SimplexJointData) -> str:
    """Create a blendPose node controlled by a simplex system, and drive the joints with it

    Parameters
    ----------
    simplex : Simplex
        A simplex system that is loaded in Maya
    jointData : SimplexJointData
        The joint data that goes along with the system

    Returns
    -------
    : str
        The blendPose node
    """
    shapeNames = [s.name for s in simplex.shapes]
    if jointData.shapeNames and jointData.shapeNames != shapeNames:
        raise ValueError("The joint data was not built for this simplex system")

    op = simplex.DCC.op
    poseNode = simplex.DCC.loadPoseNode(create=True)

    driven = jointData.drivenJoints
    joints = [jointData.jointNames[i] for i in driven]
    deltaParams = jointData.deltaParams[:, driven]

    # Only set targets for shapes that actually move a joint. The weights
    # get connected for the same indices so their dense array sizes match
    moving = np.abs(deltaParams).reshape((len(deltaParams), -1)).max(axis=1)
    shapeIdxs = np.nonzero(moving > PARAM_EPSILON)[0]

    # The blendPose works in the joint's local space relative to its actual Maya
    # parent. The delta parameters don't change with a static parent, so the
    # rest matrices are read directly from the scene
    restLocal = np.array(
        [
            _getMatrix(f"{j}.worldMatrix[0]")
            @ np.linalg.inv(_getMatrix(f"{j}.parentMatrix[0]"))
            for j in joints
        ]
    )
    targetMats, targetAAs = blendPoseTargets(restLocal, deltaParams)

    for k, mat in enumerate(restLocal):
        _setMatrix(f"{poseNode}.orig[{k}].origMat", mat)
        cmds.setAttr(f"{poseNode}.orig[{k}].origUseMat", True)

    for t in shapeIdxs:
        for k in range(len(joints)):
            pose = f"{poseNode}.target[{t}].targetPose[{k}]"
            _setMatrix(f"{pose}.targetPoseMat", targetMats[t, k])
            cmds.setAttr(f"{pose}.targetPoseAA", *targetAAs[t, k], type="double3")
            cmds.setAttr(f"{pose}.targetPoseUseMat", False)
        cmds.connectAttr(f"{op}.weights[{t}]", f"{poseNode}.weight[{t}]", force=True)

    # Move the rest transform into the offsetParentMatrix and zero everything else
    for k, joint in enumerate(joints):
        cmds.connectAttr(
            f"{poseNode}.output[{k}].outputLog",
            f"{joint}.offsetParentMatrix",
            force=True,
        )
        for attr in ("translate", "rotate", "jointOrient", "rotateAxis", "shear"):
            cmds.setAttr(f"{joint}.{attr}", 0, 0, 0)
        cmds.setAttr(f"{joint}.scale", 1, 1, 1)

    return poseNode


def bindSkin(
    mesh: str, joints: list[str], weights: np.ndarray, name: str | None = None
) -> str:
    """Bind a mesh to joints with the given weights

    Parameters
    ----------
    mesh : str
        The mesh to bind
    joints : [str, ...]
        The skin influences
    weights : np.ndarray
        The (N, J) skin weights
    name : str, optional
        The name of the skinCluster

    Returns
    -------
    : str
        The skinCluster node
    """
    kwargs = {"name": name} if name else {}
    skin = cmds.skinCluster(
        joints,
        mesh,
        toSelectedBones=True,
        bindMethod=0,
        skinMethod=0,
        normalizeWeights=1,
        obeyMaxInfluences=False,
        **kwargs,
    )[0]

    sel = om2.MSelectionList()
    sel.add(skin)
    sel.add(mesh)
    skinFn = oma2.MFnSkinCluster(sel.getDependNode(0))
    meshPath = sel.getDagPath(1).extendToShape()

    compFn = om2.MFnSingleIndexedComponent()
    comp = compFn.create(om2.MFn.kMeshVertComponent)
    compFn.setCompleteData(len(weights))

    # Match up our joint order with the skinCluster's influence order
    jointIdx = {n: i for i, n in enumerate(cmds.ls(joints, long=True))}
    infPaths = skinFn.influenceObjects()
    order = [jointIdx[p.fullPathName()] for p in infPaths]
    infIdxs = om2.MIntArray(list(range(len(infPaths))))
    flatWeights = om2.MDoubleArray(weights[:, order].flatten().tolist())
    skinFn.setWeights(meshPath, comp, infIdxs, flatWeights, False)
    return skin


def buildJoints(jointData: SimplexJointData) -> list[str]:
    """Build the joint hierarchy stored in the joint data, in its rest pose

    Parameters
    ----------
    jointData : SimplexJointData
        The joint data to build the joints from

    Returns
    -------
    : [str, ...]
        The full paths of the new joints, in the same order as the joint data
    """
    existing = [j for j in jointData.jointNames if cmds.objExists(j)]
    if existing:
        raise ValueError(f"Joints already exist: {existing}")

    # Build parents before their children
    parents = jointData.parents
    order: list[int] = []
    done: set[int] = set()

    def visit(i: int) -> None:
        if i in done:
            return
        if parents[i] >= 0:
            visit(parents[i])
        done.add(i)
        order.append(i)

    for i in range(len(parents)):
        visit(i)

    nodes: list[str] = [""] * len(parents)
    for i in order:
        cmds.select(clear=True)
        joint = cmds.joint(name=jointData.jointNames[i])
        if parents[i] >= 0:
            joint = cmds.parent(joint, nodes[parents[i]])[0]
            local = jointData.jointMats[i] @ np.linalg.inv(
                jointData.jointMats[parents[i]]
            )
        else:
            local = jointData.jointMats[i]
        # The world matrices are plain products of the locals, so parent scale
        # has to propagate like a regular transform
        cmds.setAttr(f"{joint}.segmentScaleCompensate", False)
        cmds.xform(joint, matrix=local.flatten().tolist(), objectSpace=True)
        nodes[i] = cmds.ls(joint, long=True)[0]
    cmds.select(clear=True)
    return nodes


def buildJointRig(
    smpxPath: str,
    jointData: SimplexJointData,
    mesh: str | None = None,
    name: str | None = None,
    pBar=None,
) -> Simplex:
    """Build a rig where a simplex node drives a blendShape and a blendPose

    Parameters
    ----------
    smpxPath : str
        The .smpx file containing the residual shapes
    jointData : SimplexJointData
        The joint data that goes along with the .smpx file.
        If none of the joints exist in the scene, they are built from the joint data.
        Otherwise they must all exist, in their rest pose
    mesh : str, optional
        The mesh to build the system on. Defaults to building a new one from the .smpx
    name : str, optional
        The name of the system. Defaults to the name stored in the .smpx
    pBar : QProgressDialog, optional
        An optional progress dialog

    Returns
    -------
    : Simplex
        The new simplex system
    """
    missing = [j for j in jointData.jointNames if not cmds.objExists(j)]
    if len(missing) == len(jointData.jointNames):
        buildJoints(jointData)
    elif missing:
        raise ValueError(f"Missing joints: {missing}")
    if mesh is not None and cmds.ls(cmds.listHistory(mesh), type="skinCluster"):
        raise ValueError(f"{mesh} is already skinned")

    sceneMats = getJointMatrices(jointData.jointNames)
    if not np.allclose(sceneMats, jointData.jointMats, atol=1e-5):
        raise ValueError(
            "The joints in the scene are not in the rest pose of the joint data"
        )

    simplex = Simplex.buildSystemFromSmpx(smpxPath, thing=mesh, name=name, pBar=pBar)
    setupBlendPose(simplex, jointData)
    bindSkin(
        simplex.DCC.mesh,
        jointData.jointNames,
        jointData.weights,
        name=f"{simplex.name}_SC",
    )
    return simplex
