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
"""Read the JSON file that describes a layered joint solve

The file looks like this::

    {
        "hierarchy": [
            ["neck01", null],
            ["neck02", "neck01"],
            ["head01", "neck02"],
            ...
        ],
        "settings": {"rotRegScale": 0.5, "iterations": 10, "minWeight": 0.3},
        "jobs": {
            "bones": [
                {
                    "name": "bones",
                    "rotReg": 1.0,
                    "joints": {"FACIAL_C_Jaw": "rt", "FACIAL_C_LowerLipRotation": "r"}
                }
            ],
            "volume": [ ...passes... ],
            "rolls": [ ...passes... ]
        },
        "defaultJobs": ["bones", "volume"],
        "shapeJobs": {
            "lipsUpperRollIn_X": ["bones", "volume", "rolls"]
        }
    }

* ``jobs`` maps a job name to its list of passes. The passes of a job run in order,
  and each one solves its joints starting from where the previous pass left them.
  The joint flags say whether a joint can rotate (``r``), translate (``t``), or both.
  ``rotReg`` pulls the rotations toward where they started (see
  `jointPoses.SolvePass`)
* ``defaultJobs`` are the jobs every shape runs, in order
* ``shapeJobs`` replaces the default jobs for specific simplex shapes, by name
* ``settings`` are passed to `jointPoses.solveLayeredPoses`
  (and ``weightRounds`` to `simplexJoints.jointifySimplexLayered`).
  ``"poseSolver": "rigid"`` uses `jointPosesRigid.solveLayeredPoses` instead, which
  also takes ``rotRadius`` and ``priorReg``, and respects the ``r``/``t`` flags

Joints that aren't in the skeleton being solved are skipped, so the same file
works with a reduced skeleton
"""

from __future__ import annotations

import json

from .jointPoses import SolveJob, SolvePass


def solveKwargsFromConfig(rawconfig: dict | str) -> dict:
    """Get the keyword arguments for `simplexJoints.jointifySmpx` (or
    `mayaJointRig.jointifySmpxFromScene`) from a solve config

    Parameters
    ----------
    config : dict or str
        The config, or the path to its file

    Returns
    -------
    : dict
        The "jointNames", "parents", "lockJoints", "jobs" and "shapeJobs" keyword
        arguments, plus the settings
    """
    if isinstance(rawconfig, str):
        with open(rawconfig) as f:
            config = json.load(f)
    else:
        config = rawconfig

    kwargs = dict(config.get("settings", {}))
    jointNames, pars = zip(*config["hierarchy"])
    indexByName = {k: i for i, k in enumerate(jointNames)}
    parIdxs = [indexByName.get(p, -1) for p in pars]
    locks = config.get("locks", [])
    lockIdxs = [indexByName.get(p, -1) for p in locks]

    kwargs['jointNames'] = jointNames
    kwargs['parents'] = parIdxs
    kwargs['lockJoints'] = lockIdxs

    jobs, jobIndexByName = [], {}
    missing = set()
    for jobName, passes in config["jobs"].items():
        solvePasses = []
        for p in passes:
            joints = {}
            for name, flags in p["joints"].items():
                if name not in indexByName:
                    missing.add(name)
                    continue
                joints[indexByName[name]] = ("r" in flags, "t" in flags)
            solvePasses.append(
                SolvePass(p.get("name", jobName), joints, p.get("rotReg", 0.5))
            )
        jobIndexByName[jobName] = len(jobs)
        jobs.append(SolveJob(jobName, solvePasses))
    if missing:
        print(
            f"Skipping {len(missing)} joints from the config that aren't being solved"
        )

    def lookup(names):
        bad = [n for n in names if n not in jobIndexByName]
        if bad:
            raise ValueError(f"Unknown jobs in the solve config: {bad}")
        return [jobIndexByName[n] for n in names]

    defaultJobs = lookup(config["defaultJobs"])
    overrides = {
        shape: lookup(names) for shape, names in config.get("shapeJobs", {}).items()
    }

    def shapeJobs(simplex, shapes):
        known = {s.name for s in shapes}
        unknown = sorted(set(overrides) - known)
        if unknown:
            print(
                f"The solve config has jobs for shapes that aren't being solved: {unknown}"
            )
        return [overrides.get(s.name, defaultJobs) for s in shapes]

    kwargs["jobs"] = jobs
    kwargs["shapeJobs"] = shapeJobs

    return kwargs
