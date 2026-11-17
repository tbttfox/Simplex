"""Run the layered joint solve on a .smpx file, outside of Maya

Example::

    python scripts/jointifyLayered.py face.smpx face_residual.smpx ^
        --joint-mats jointMats.npy --config jointSolveConfig.json

The config file holds the joint hierarchy, the solve jobs, which shapes run them, and
the solve settings. See simplexui/commands/jointSolveConfig.py for its format. Any
setting passed on the command line overrides the one in the config

Outputs:
    <out>.smpx         The residual shapes
    <out>.npz          The SimplexJointData (weights, blendPose deltas, etc)
    <out>_poses.npz    The joint matrices for every simplex shape:
                         jointNames, parents, restMats (J, 4, 4)
                         shapeNames
                         worldMats (numShapes, J, 4, 4)     world matrices at each shape
                         localMats (numShapes, J, 4, 4)     local matrices at each shape
                         targetPoseMat (numShapes, J, 4, 4) blendPose targetPoseMat values
                         targetPoseAA (numShapes, J, 3)     blendPose targetPoseAA values
                         weights (N, J)
"""

from __future__ import annotations

import argparse
import datetime
import os
import time
import numpy as np
from pysimplex import PySimplex

from .alembicCommon import readSmpx
from .applyCorrectives import _buildSolverInputs
from .jointPoses import blendPoseTargets, localMats
from .jointSolveConfig import solveKwargsFromConfig
from .simplexJoints import jointifySmpx
from ..items import Simplex


def writePoses(smpxPath: str, jointData, outPath: str) -> None:
    """Write the joint matrices at every simplex shape"""
    jsString = readSmpx(smpxPath)[0]
    simplex = Simplex.buildSystemFromJsonString(jsString, None, forceDummy=True)
    solver = PySimplex(jsString)
    indexBySlider = {s: i for i, s in enumerate(simplex.sliders)}

    shapeVecs = np.zeros((len(simplex.shapes), len(simplex.shapes)))
    for item in list(simplex.sliders) + list(simplex.combos):
        for pair in item.prog.pairs:
            if pair.shape.isRest:
                continue
            inVec = _buildSolverInputs(simplex, item, pair.value, indexBySlider)
            shapeVecs[simplex.shapes.index(pair.shape)] = solver.solve(inVec)

    world = jointData.evaluate(shapeVecs)
    targetMats, targetAAs = blendPoseTargets(jointData.restLocal, jointData.deltaParams)
    np.savez(
        outPath,
        jointNames=np.array(jointData.jointNames),
        parents=np.array(jointData.parents, dtype=int),
        restMats=jointData.jointMats,
        shapeNames=np.array([s.name for s in simplex.shapes]),
        worldMats=world,
        localMats=localMats(world, jointData.parents),
        targetPoseMat=targetMats,
        targetPoseAA=targetAAs,
        weights=jointData.weights,
    )


def jointifyLayered(
    inSmpx: str,
    outSmpx: str,
    config: dict,
    joint_mats: str,
    weights: str | None = None,
    **settings,
):
    """Interface function for doing a layered jointify solve

    Any settings that aren't None override the ones in the config
    """
    start = time.time()
    mats = np.load(joint_mats).astype(np.float64)

    kwargs = solveKwargsFromConfig(config)
    kwargs.update({k: v for k, v in settings.items() if v is not None})
    if weights:
        kwargs["weights"] = np.load(weights)

    outBase = os.path.splitext(outSmpx)[0]
    jointData = jointifySmpx(
        inSmpx,
        outSmpx,
        mats,
        jointDataPath=outBase + ".npz",
        **kwargs,
    )
    writePoses(inSmpx, jointData, outBase + "_poses.npz")
    print(f"\nWrote {outSmpx}, {outBase}.npz, {outBase}_poses.npz")

    delta = datetime.timedelta(seconds=round(time.time() - start))
    print(f"Joint fit RMSE {jointData.rmse:.5f}. Took {delta}")


def main():
    """Parse and unpack the cmd line arguments"""
    # fmt: off
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("inSmpx")
    parser.add_argument("outSmpx")
    parser.add_argument("--config", required=True, help="The solve config JSON file")
    parser.add_argument("--joint-mats", required=True, help="A .npy of (J, 4, 4) rest world matrices")
    parser.add_argument("--weights", help="A .npy of (N, J) starting skin weights")
    parser.add_argument("--weight-rounds", type=int, help="How many times to re-solve the weights. 0 keeps --weights")
    parser.add_argument("--weight-solver", choices=["demBones", "smooth"], help="How to solve the weights")
    parser.add_argument("--weight-smooth-iterations", type=int, help="How many times to smooth the final weights before building the residuals. Defaults to 3")
    parser.add_argument("--weight-smooth-limit", action="store_true", default=None, help="Keep the --max-influences limit when smoothing the final weights")
    parser.add_argument("--smoothness", type=float, help="Smooth weight solver only. How strongly the weights are smoothed")
    parser.add_argument("--max-influences", type=int, help="The max number of joints per vertex")
    parser.add_argument("--candidates", type=int, help="Smooth weight solver only. How many of each vertex's closest joints it can pick from")
    parser.add_argument("--pose-solver", choices=["demBones", "rigid"], help="Which layered solve to use")
    parser.add_argument("--rot-reg-scale", type=float, help="Multiplies each pass's rotReg. 0 turns it off")
    parser.add_argument("--rot-radius", type=float, help="Rigid solver only. The min point distance for the rotation regularization")
    parser.add_argument("--prior-reg", type=float, help="Rigid solver only. How much joints are held in place where they have little influence")
    parser.add_argument("--processes", type=int, help="Rigid solver only. How many processes solve in parallel. Defaults to 16")
    parser.add_argument("--min-weight", type=float, help="Bone weighs under this value will be pruned")
    parser.add_argument("--iterations", type=int, help="Dem Bones transformation iterations per pass")
    parser.add_argument("--chunk-size", type=int, help="How many shapes to give Dem Bones at once")
    args = parser.parse_args()
    # fmt: on

    jointifyLayered(
        args.inSmpx,
        args.outSmpx,
        args.config,
        args.joint_mats,
        args.weights,
        weightRounds=args.weight_rounds,
        weightSolver=args.weight_solver,
        weightSmoothIterations=args.weight_smooth_iterations,
        weightSmoothLimit=args.weight_smooth_limit,
        smoothness=args.smoothness,
        maxInfluences=args.max_influences,
        candidates=args.candidates,
        minWeight=args.min_weight,
        iterations=args.iterations,
        chunkSize=args.chunk_size,
        poseSolver=args.pose_solver,
        rotRegScale=args.rot_reg_scale,
        rotRadius=args.rot_radius,
        priorReg=args.prior_reg,
        processes=args.processes,
    )


if __name__ == "__main__":
    main()
