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

#pragma once

#include <maya/MEvaluationNode.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnStringData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MVector.h>

#include <unordered_map>

class blendPose : public MPxNode {
   public:
    static void* creator();
    static MStatus initialize();
    virtual MStatus compute(const MPlug& plug, MDataBlock& data);

    void getAllData(
        MDataBlock& dataBlock,
        std::vector<MVector>& restAAs,               // aOrigAxisAngle
        std::vector<MMatrix>& restMats,              // aOrigMatrix
        std::vector<bool>& restUseMats,              // aOrigUseMatrix
        std::vector<std::vector<MVector>>& tarAAs,   // aTargetPoseAxisAngle
        std::vector<std::vector<MMatrix>>& tarMats,  // aTargetPoseMatrix
        std::vector<std::vector<bool>>& tarUseMats,  // aTargetPoseUseMatrix
        std::vector<float>& weights                  // aWeight
    );

    void setAllData(
        MDataBlock& dataBlock, const std::vector<MMatrix>& outLinMats,
        const std::vector<MMatrix>& outLogMats, const std::vector<MVector>& outAAs
    );

    template <typename T>
    std::vector<T> computeData(
        std::vector<std::vector<T>> targets, std::vector<float> weights, T zero, T offset,
        bool useOffset
    );

    void decomposeMatList(
        const std::vector<MMatrix>& allmats, const std::vector<MVector>& allAAs,
        const std::vector<bool>& allUseMats, std::vector<MVector>& restScale,
        std::vector<MVector>& restShear, std::vector<MVector>& restAAs,
        std::vector<MVector>& restTran
    );

   public:
    static MObject aOutput;
    static MObject aOutputAxisAngle;
    static MObject aOutputLinearMatrix;
    static MObject aOutputLogMatrix;

    static MObject aOrig;
    static MObject aOrigAxisAngle;
    static MObject aOrigMatrix;
    static MObject aOrigUseMatrix;

    static MObject aTarget;
    static MObject aTargetPose;
    static MObject aTargetPoseAxisAngle;
    static MObject aTargetPoseMatrix;
    static MObject aTargetPoseUseMatrix;

    static MObject aWeight;

    static MTypeId id;
    static MString typeName;

   private:
};
