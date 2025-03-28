
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

class eulerToAxisAngle : public MPxNode {
   public:
    static void* creator();
    static MStatus initialize();
    virtual MStatus compute(const MPlug& plug, MDataBlock& data);

   public:
    static MObject aInput;
    static MObject aInputX;
    static MObject aInputY;
    static MObject aInputZ;
    static MObject aInputOrder;
    static MObject aOutput;

    static MTypeId id;
    static MString typeName;
};

class axisAngleToEuler : public MPxNode {
   public:
    static void* creator();
    static MStatus initialize();
    virtual MStatus compute(const MPlug& plug, MDataBlock& data);

   public:
    static MObject aInput;
    static MObject aInputOrder;
    static MObject aOutput;
    static MObject aOutputX;
    static MObject aOutputY;
    static MObject aOutputZ;

    static MTypeId id;
    static MString typeName;
};
