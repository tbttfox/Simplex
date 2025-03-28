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

#include <unordered_map>

class blendPose : public MPxNode {
   public:
    static void* creator();
    static MStatus initialize();
    virtual MStatus compute(const MPlug& plug, MDataBlock& data);
    void getAllData(
        MDataBlock& dataBlock,
        std::unordered_map<int, std::tuple<int, float, std::vector<MMatrix>>>& targets,
        std::vector<MMatrix>& restPose
    );

    void setAllData(
        const MPlug& plug, MDataBlock& dataBlock, std::vector<MMatrix>& rawMats,
        std::vector<MMatrix>& quatMats
    );

    void computeRawMats(
        const std::unordered_map<int, std::tuple<int, float, std::vector<MMatrix>>>& targets,
        const std::vector<MMatrix>& restPose, std::vector<MMatrix>& rawMats
    );

    void computeQuatMats(
        const std::unordered_map<int, std::tuple<int, float, std::vector<MMatrix>>>& targets,
        const std::vector<MMatrix>& restPose, std::vector<MMatrix>& QuatMats
    );

   public:
    static MObject aOutputPose;
    static MObject aOutputRawPose;
    static MObject aOriginalPose;
    static MObject aInputTarget;
    static MObject aInputTargetPose;
    static MObject aInputTargetLevel;
    static MObject aWeight;

    static MTypeId id;
    static MString typeName;

   private:
};
