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
#include "eulerAxisAngle.h"

#include <maya/MEulerRotation.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>

#include <vector>

#include "matutils.h"

MObject eulerToAxisAngle::aInput;
MObject eulerToAxisAngle::aInputX;
MObject eulerToAxisAngle::aInputY;
MObject eulerToAxisAngle::aInputZ;
MObject eulerToAxisAngle::aInputOrder;
MObject eulerToAxisAngle::aOutput;
MTypeId eulerToAxisAngle::id(0x00122717);
MString eulerToAxisAngle::typeName("eulerToAxisAngle");

MStatus eulerToAxisAngle::compute(const MPlug& plug, MDataBlock& dataBlock) {
    if (plug == aOutput) {
        MDataHandle inHandle = dataBlock.inputValue(aInput);
        double rx = inHandle.child(aInputX).asDouble();
        double ry = inHandle.child(aInputY).asDouble();
        double rz = inHandle.child(aInputZ).asDouble();

        short rotOrderIndex = dataBlock.inputValue(aInputOrder).asShort();
        auto rotOrder = static_cast<MEulerRotation::RotationOrder>(rotOrderIndex);

        MEulerRotation rot(rx, ry, rz, rotOrder);
        MVector aa = logm(rot.asMatrix());
        MDataHandle outHandle = dataBlock.outputValue(aOutput);
        outHandle.set3Double(aa[0], aa[1], aa[2]);
        dataBlock.setClean(plug);
        return MStatus::kSuccess;
    }
    return MStatus::kUnknownParameter;
}
void* eulerToAxisAngle::creator() { return new eulerToAxisAngle(); }

MStatus eulerToAxisAngle::initialize() {
    MFnNumericAttribute nAttr;
    MFnUnitAttribute uAttr;
    MFnEnumAttribute eAttr;

    aInputX = uAttr.create("rotateX", "rx", MFnUnitAttribute::kAngle, 0.0);
    aInputY = uAttr.create("rotateY", "ry", MFnUnitAttribute::kAngle, 0.0);
    aInputZ = uAttr.create("rotateZ", "rz", MFnUnitAttribute::kAngle, 0.0);
    aInput = nAttr.create("rotate", "r", aInputX, aInputY, aInputZ);
    addAttribute(aInput);

    aInputOrder = eAttr.create("rotateOrder", "ro", 0);
    eAttr.addField("xyz", 0);
    eAttr.addField("yzx", 1);
    eAttr.addField("zxy", 2);
    eAttr.addField("xzy", 3);
    eAttr.addField("yxz", 4);
    eAttr.addField("zyx", 5);
    eAttr.setKeyable(true);
    addAttribute(aInputOrder);

    aOutput = nAttr.create("output", "o", MFnNumericData::k3Double);
    nAttr.setWritable(false);
    addAttribute(aOutput);

    std::vector<MObject*> inputs = {
        &aInput, &aInputX, &aInputY, &aInputZ, &aInputOrder,
    };

    std::vector<MObject*> outputs = {
        &aOutput,
    };

    for (auto inat : inputs) {
        for (auto outat : outputs) {
            attributeAffects(*inat, *outat);
        }
    }

    return MStatus::kSuccess;
}

MObject axisAngleToEuler::aInput;
MObject axisAngleToEuler::aInputOrder;
MObject axisAngleToEuler::aOutput;
MObject axisAngleToEuler::aOutputX;
MObject axisAngleToEuler::aOutputY;
MObject axisAngleToEuler::aOutputZ;
MTypeId axisAngleToEuler::id(0x00122718);
MString axisAngleToEuler::typeName("axisAngleToEuler");

MStatus axisAngleToEuler::compute(const MPlug& plug, MDataBlock& dataBlock) {
    if (plug == aOutput or plug == aOutputX or plug == aOutputY or plug == aOutputZ) {
        MVector aa = dataBlock.inputValue(aInput).asVector();
        MMatrix rmat = expm(aa);
        short rotOrderIndex = dataBlock.inputValue(aInputOrder).asShort();

        auto rotOrder = static_cast<MEulerRotation::RotationOrder>(rotOrderIndex);
        MEulerRotation eul = MEulerRotation::decompose(rmat, rotOrder);

        MDataHandle inHandle = dataBlock.outputValue(aOutput);
        inHandle.child(aOutputX).setDouble(eul.x);
        inHandle.child(aOutputY).setDouble(eul.y);
        inHandle.child(aOutputZ).setDouble(eul.z);
        dataBlock.setClean(aOutput);
        dataBlock.setClean(aOutputX);
        dataBlock.setClean(aOutputY);
        dataBlock.setClean(aOutputZ);
        return MStatus::kSuccess;
    }
    return MStatus::kUnknownParameter;
}
void* axisAngleToEuler::creator() { return new axisAngleToEuler(); }

MStatus axisAngleToEuler::initialize() {
    MFnNumericAttribute nAttr;
    MFnUnitAttribute uAttr;
    MFnEnumAttribute eAttr;

    aOutputX = uAttr.create("rotateX", "rx", MFnUnitAttribute::kAngle, 0.0);
    aOutputY = uAttr.create("rotateY", "ry", MFnUnitAttribute::kAngle, 0.0);
    aOutputZ = uAttr.create("rotateZ", "rz", MFnUnitAttribute::kAngle, 0.0);
    aOutput = nAttr.create("rotate", "r", aOutputX, aOutputY, aOutputZ);
    addAttribute(aOutput);

    aInputOrder = eAttr.create("rotateOrder", "ro", 0);
    eAttr.addField("xyz", 0);
    eAttr.addField("yzx", 1);
    eAttr.addField("zxy", 2);
    eAttr.addField("xzy", 3);
    eAttr.addField("yxz", 4);
    eAttr.addField("zyx", 5);
    addAttribute(aInputOrder);

    aInput = nAttr.create("input", "i", MFnNumericData::k3Double);
    addAttribute(aInput);

    std::vector<MObject*> inputs = {
        &aOutput,
        &aOutputX,
        &aOutputY,
        &aOutputZ,
    };
    std::vector<MObject*> outputs = {
        &aInput,
        &aInputOrder,
    };
    for (auto inat : inputs) {
        for (auto outat : outputs) {
            attributeAffects(*inat, *outat);
        }
    }

    return MStatus::kSuccess;
}
