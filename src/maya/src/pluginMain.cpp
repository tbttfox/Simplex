/*
Copyright 2016, Blur Studio

This file is part of Simplex.

Simplex is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Simplex is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with Simplex.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <maya/MFnPlugin.h>
#include <maya/MObject.h>
#include <maya/MStatus.h>

#include "basicBlendShape.h"
#include "blendPose.h"
#include "eulerAxisAngle.h"
#include "simplex_mayaNode.h"
#include "version.h"

MStatus initializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj, "Blur Studio", VERSION_STRING, "Any");

    status = plugin.registerNode(
        "simplex_maya", simplex_maya::id, &simplex_maya::creator, &simplex_maya::initialize
    );

    if (!status) {
        status.perror("registerNode simplex_maya");
        return status;
    }

    status = plugin.registerNode(
        "basicBlendShape", basicBlendShape::id, &basicBlendShape::creator,
        &basicBlendShape::initialize, MPxNode::kBlendShape
    );

    if (!status) {
        status.perror("registerNode basicBlendShape");
        return status;
    }

    status = plugin.registerNode(
        blendPose::typeName, blendPose::id, &blendPose::creator, &blendPose::initialize,
        MPxNode::kDependNode
    );

    if (!status) {
        status.perror("registerNode blendPose");
        return status;
    }

    status = plugin.registerNode(
        eulerToAxisAngle::typeName, eulerToAxisAngle::id, &eulerToAxisAngle::creator,
        &eulerToAxisAngle::initialize, MPxNode::kDependNode
    );

    if (!status) {
        status.perror("registerNode eulerToAxisAngle");
        return status;
    }

    status = plugin.registerNode(
        axisAngleToEuler::typeName, axisAngleToEuler::id, &axisAngleToEuler::creator,
        &axisAngleToEuler::initialize, MPxNode::kDependNode
    );

    if (!status) {
        status.perror("registerNode axisAngleToEuler");
        return status;
    }

    return status;
}

MStatus uninitializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj);

    status = plugin.deregisterNode(simplex_maya::id);
    if (!status) {
        status.perror("deregisterNode simplex_maya");
        return status;
    }

    status = plugin.deregisterNode(basicBlendShape::id);
    if (!status) {
        status.perror("deregisterNode basicBlendShape");
        return status;
    }

    status = plugin.deregisterNode(blendPose::id);
    if (!status) {
        status.perror("deregisterNode blendPose");
        return status;
    }

    status = plugin.deregisterNode(eulerToAxisAngle::id);
    if (!status) {
        status.perror("deregisterNode eulerToAxisAngle");
        return status;
    }

    status = plugin.deregisterNode(axisAngleToEuler::id);
    if (!status) {
        status.perror("deregisterNode axisAngleToEuler");
        return status;
    }

    return status;
}
