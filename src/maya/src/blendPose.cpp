#include "blendPose.h"
#include "matutils.h"

#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <maya/MQuaternion.h>
#include <maya/MVector.h>

#include <cmath>
#include <map>
#include <numbers>
#include <tuple>
#include <unordered_map>

#include "Eigen/Dense"
#include "mayaIterators.h"

#define CHECKSTATAFFECTS(s)             \
    if (!(s)) {                         \
        (s).perror("attributeAffects"); \
        return (s);                     \
    }
#define CHECKSTATCREATE(s)               \
    if (!(s)) {                          \
        (s).perror("attributeCreation"); \
        return (s);                      \
    }
#define CHECKSTATADDATTR(s)         \
    if (!(s)) {                     \
        (s).perror("addAttribute"); \
        return (s);                 \
    }

// For linux and apple
typedef unsigned int UINT;

MTypeId blendPose::id(0x00122716);
MString blendPose::typeName("blendPose");

// Attributes
MObject blendPose::aOutput;
MObject blendPose::aOutputAxisAngle;
MObject blendPose::aOutputLinearMatrix;
MObject blendPose::aOutputLogMatrix;

MObject blendPose::aOrig;
MObject blendPose::aOrigAxisAngle;
MObject blendPose::aOrigMatrix;
MObject blendPose::aOrigUseMatrix;

MObject blendPose::aTarget;
MObject blendPose::aTargetPose;
MObject blendPose::aTargetPoseAxisAngle;
MObject blendPose::aTargetPoseMatrix;
MObject blendPose::aTargetPoseUseMatrix;

MObject blendPose::aWeight;


void* blendPose::creator() { return new blendPose(); }

void blendPose::decomposeMatList(
    const std::vector<MMatrix>& allmats, const std::vector<MVector>& allAAs,
    const std::vector<bool>& allUseMats, std::vector<MVector>& restScale,
    std::vector<MVector>& restShear, std::vector<MVector>& restAAs, std::vector<MVector>& restTran
) {
    std::vector<MMatrix> restRot;

    for (size_t i = 0; i < allmats.size(); ++i) {
        const auto& m = allmats[i];

        MVector rsc, rsh, rt;
        MMatrix rr;
        decompose(m, rsc, rsh, rr, rt);

        restScale.push_back(rsc);
        restShear.push_back(rsh);
        restTran.push_back(rt);
        if (allUseMats[i]) {
            restAAs.push_back(logm(rr));
        } else {
            restAAs.push_back(allAAs[i]);
        }
    }
}

MStatus blendPose::initialize() {
    MStatus status;

    MFnNumericAttribute nAttr;
    MFnMatrixAttribute mAttr;
    MFnCompoundAttribute cAttr;

    aOutputAxisAngle = nAttr.create("outputAA", "oaa", MFnNumericData::k3Double, 0.0, &status);
    CHECKSTATCREATE(status);
    nAttr.setWritable(false);
    aOutputLinearMatrix = mAttr.create("outputLin", "oln", MFnMatrixAttribute::kDouble, &status);
    CHECKSTATCREATE(status);
    nAttr.setWritable(false);
    aOutputLogMatrix = mAttr.create("outputLog", "olg", MFnMatrixAttribute::kDouble, &status);
    CHECKSTATCREATE(status);
    nAttr.setWritable(false);

    aOutput = cAttr.create("output", "out", &status);
    CHECKSTATCREATE(status);
    cAttr.addChild(aOutputAxisAngle);
    cAttr.addChild(aOutputLinearMatrix);
    cAttr.addChild(aOutputLogMatrix);
    cAttr.setArray(true);
    cAttr.setWritable(false);
    cAttr.setUsesArrayDataBuilder(true);
    status = addAttribute(aOutput);
    CHECKSTATADDATTR(status);

    aOrigAxisAngle = nAttr.create("origAA", "gaa", MFnNumericData::k3Double, 0.0, &status);
    CHECKSTATCREATE(status);
    aOrigMatrix = mAttr.create("origMat", "gm", MFnMatrixAttribute::kDouble, &status);
    CHECKSTATCREATE(status);
    aOrigUseMatrix = nAttr.create("origUseMat", "gu", MFnNumericData::kBoolean, true, &status);
    CHECKSTATCREATE(status);

    aOrig = cAttr.create("orig", "og", &status);
    CHECKSTATCREATE(status);
    cAttr.addChild(aOrigAxisAngle);
    cAttr.addChild(aOrigMatrix);
    cAttr.addChild(aOrigUseMatrix);
    cAttr.setArray(true);
    status = addAttribute(aOrig);
    CHECKSTATADDATTR(status);

    aTargetPoseAxisAngle =
        nAttr.create("targetPoseAA", "taa", MFnNumericData::k3Double, 0.0, &status);
    CHECKSTATCREATE(status);
    aTargetPoseMatrix = mAttr.create("targetPoseMat", "tpm", MFnMatrixAttribute::kDouble, &status);
    CHECKSTATCREATE(status);
    aTargetPoseUseMatrix =
        nAttr.create("targetPoseUseMat", "tpu", MFnNumericData::kBoolean, false, &status);
    CHECKSTATCREATE(status);

    aTargetPose = cAttr.create("targetPose", "tp", &status);
    CHECKSTATCREATE(status);
    cAttr.addChild(aTargetPoseAxisAngle);
    cAttr.addChild(aTargetPoseMatrix);
    cAttr.addChild(aTargetPoseUseMatrix);
    cAttr.setArray(true);

    aTarget = cAttr.create("target", "t", &status);
    CHECKSTATCREATE(status);
    cAttr.addChild(aTargetPose);
    cAttr.setArray(true);
    status = addAttribute(aTarget);
    CHECKSTATADDATTR(status);

    aWeight = nAttr.create("weight", "w", MFnNumericData::kDouble, 0.0, &status);
    CHECKSTATCREATE(status);
    nAttr.setSoftMin(0.0);
    nAttr.setSoftMax(1.0);
    nAttr.setArray(true);
    nAttr.setKeyable(true);
    status = addAttribute(aWeight);
    CHECKSTATADDATTR(status);

    std::vector<MObject*> inputs = {
        &aOrig,       &aOrigAxisAngle,       &aOrigMatrix,       &aOrigUseMatrix,       &aTarget,
        &aTargetPose, &aTargetPoseAxisAngle, &aTargetPoseMatrix, &aTargetPoseUseMatrix, &aWeight,
    };
    std::vector<MObject*> outputs = {
        &aOutput,
        &aOutputAxisAngle,
        &aOutputLinearMatrix,
        &aOutputLogMatrix,
    };

    for (auto inat : inputs) {
        for (auto outat : outputs) {
            status = attributeAffects(*inat, *outat);
            CHECKSTATAFFECTS(status);
        }
    }

    return MS::kSuccess;
}

void blendPose::getAllData(
    MDataBlock& dataBlock,
    std::vector<MVector>& restAAs,               // aOrigAxisAngle
    std::vector<MMatrix>& restMats,              // aOrigMatrix
    std::vector<bool>& restUseMats,              // aOrigUseMatrix
    std::vector<std::vector<MVector>>& tarAAs,   // aTargetPoseAxisAngle
    std::vector<std::vector<MMatrix>>& tarMats,  // aTargetPoseMatrix
    std::vector<std::vector<bool>>& tarUseMats,  // aTargetPoseUseMatrix
    std::vector<float>& weights                  // aWeight
) {
    MArrayDataHandle origAH = dataBlock.inputArrayValue(aOrig);
    int prevIdx = 0;
    for (auto [index, handle] : MArrayInputDataHandleRange(origAH)) {
        for (; prevIdx < index; ++prevIdx) {
            restMats.push_back(MMatrix());
            restAAs.push_back(MVector());
            restUseMats.push_back(true);
        }
        restMats.push_back(handle.child(aOrigMatrix).asMatrix());
        restAAs.push_back(handle.child(aOrigAxisAngle).asVector());
        restUseMats.push_back(handle.child(aOrigUseMatrix).asBool());
        prevIdx = index + 1;
    }

    MArrayDataHandle weightAH = dataBlock.inputArrayValue(aWeight);
    prevIdx = 0;
    for (auto [index, handle] : MArrayInputDataHandleRange(weightAH)) {
        for (; prevIdx < index; ++prevIdx) {
            weights.push_back(0.0);
        }
        weights.push_back(handle.asDouble());
        prevIdx = index + 1;
    }

    MArrayDataHandle targetAH = dataBlock.inputArrayValue(aTarget);
    int prevTarIndex = 0;

    for (auto [tarindex, tarhandle] : MArrayInputDataHandleRange(targetAH)) {
        std::vector<MVector> poseAAs;
        std::vector<MMatrix> poseMats;
        std::vector<bool> poseUseMats;

        for (; prevTarIndex < tarindex; ++prevTarIndex) {
            poseMats = restMats;
            poseAAs = restAAs;
            poseUseMats.resize(restMats.size());
        }

        MArrayDataHandle ah = tarhandle;
        prevIdx = 0;
        for (auto [index, handle] : MArrayInputDataHandleRange(ah)) {
            for (; prevIdx < index; ++prevIdx) {
                poseMats.push_back(MMatrix());
                poseUseMats.push_back(false);
                poseAAs.push_back(MVector());
            }
            poseMats.push_back(handle.child(aTargetPoseMatrix).asMatrix());
            poseUseMats.push_back(handle.child(aTargetPoseUseMatrix).asBool());
            poseAAs.push_back(handle.child(aTargetPoseAxisAngle).asVector());

            prevIdx = index + 1;
        }
        prevTarIndex = tarindex + 1;
    }
}

void blendPose::setAllData(
    MDataBlock& dataBlock, const std::vector<MMatrix>& outLinMats,
    const std::vector<MMatrix>& outLogMats, const std::vector<MVector>& outAAs
) {
    MArrayDataHandle outAH = dataBlock.outputArrayValue(aOutput);
    MArrayDataBuilder builder = outAH.builder();
    for (auto [handle, key] : MArrayOutputDataHandleRange(builder, outLinMats)) {
        handle.child(aOutputAxisAngle).setMVector(outAAs[key]);
        handle.child(aOutputLinearMatrix).setMMatrix(outLinMats[key]);
        handle.child(aOutputLogMatrix).setMMatrix(outLogMats[key]);
    }
}

template <typename T>
std::vector<T> blendPose::computeData(
    std::vector<std::vector<T>> targets, std::vector<float> weights, T zero, T offset,
    bool useOffset
) {
    std::vector<T> ret;
    ret.resize(targets[0].size(), zero);
    float allweight = 0.0f;

    for (size_t poseIdx = 0; poseIdx < weights.size(); ++poseIdx) {
        float& weight = weights[poseIdx];
        if (fabs(weight) < 1e-12) {
            continue;
        }
        allweight += fabs(weight);
        for (size_t matIdx = 0; matIdx < targets[poseIdx].size(); ++matIdx) {
            auto& mat = targets[poseIdx][matIdx];
            if (useOffset) {
                mat -= offset;
            }
            ret[matIdx] += weight * mat;
        }
    }

    if (allweight == 0.0) {
        ret.resize(0);
        ret.resize(targets[0].size(), offset);
        return ret;
    }
    if (useOffset) {
        for (auto& r : ret) {
            r += offset;
        }
    }
    return ret;
}

MStatus blendPose::compute(const MPlug& plug, MDataBlock& dataBlock) {
    if (!(plug == aOutput || plug == aOutputAxisAngle || plug == aOutputLinearMatrix ||
          plug == aOutputLogMatrix)) {
        return MStatus::kUnknownParameter;
    }

    std::vector<MVector> restRawAAs;
    std::vector<MMatrix> restMats;
    std::vector<bool> restUseMats;
    std::vector<std::vector<MVector>> tarAAPoses;
    std::vector<std::vector<MMatrix>> tarPoses;
    std::vector<std::vector<bool>> tarUseMats;
    std::vector<float> weights;
    getAllData(
        dataBlock, restRawAAs, restMats, restUseMats, tarAAPoses, tarPoses, tarUseMats, weights
    );

    if (tarPoses.empty()) {
        std::vector<MVector> zm;
        zm.resize(restMats.size());
        setAllData(dataBlock, restMats, restMats, zm);
        return MStatus::kSuccess;
    }

    std::vector<MVector> restScale, restShear, restTran, restAAs;
    decomposeMatList(restMats, restRawAAs, restUseMats, restScale, restShear, restAAs, restTran);

    std::vector<MMatrix> restInvMats;
    for (const auto& m : restMats) {
        restInvMats.push_back(m.inverse());
    }

    std::vector<std::vector<MMatrix>> tarLocalMats;
    std::vector<std::vector<MVector>> tarTrans, tarScales, tarShears, tarAAs;

    for (size_t i = 0; i < tarPoses.size(); ++i) {
        auto& tarMats = tarPoses[i];
        auto& tarAAPose = tarAAPoses[i];
        auto& useMats = tarUseMats[i];
        std::vector<MMatrix> tarInRestMats;
        tarInRestMats.reserve(tarMats.size());
        for (size_t j = 0; j < tarMats.size(); ++j) {
            tarInRestMats.push_back(tarMats[j] * restInvMats[j]);
        }

        std::vector<MVector> tarScale, tarShear, tarAA, tarTran;
        decomposeMatList(tarInRestMats, tarAAPose, useMats, tarScale, tarShear, tarAA, tarTran);
        tarLocalMats.push_back(tarInRestMats);
        tarTrans.push_back(tarTran);
        tarScales.push_back(tarScale);
        tarShears.push_back(tarShear);
        tarAAs.push_back(tarAA);
    }

    MMatrix zeroMat;
    zeroMat *= 0;
    MVector oneVec(1, 1, 1);
    std::vector<MMatrix> outMats = computeData(tarLocalMats, weights, zeroMat, MMatrix(), true);
    std::vector<MVector> outScales = computeData(tarScales, weights, MVector(), oneVec, true);
    std::vector<MVector> outShears = computeData(tarShears, weights, MVector(), MVector(), false);
    std::vector<MVector> outTrans = computeData(tarTrans, weights, MVector(), MVector(), false);
    std::vector<MVector> outAAs = computeData(tarAAs, weights, MVector(), MVector(), false);

    std::vector<MMatrix> outRotMats, outLogMats, outMats2;
    for (const auto& p : outAAs) {
        outRotMats.push_back(expm(p));
    }

    for (size_t i = 0; i < restMats.size(); ++i) {
        auto& sc = outScales[i];
        auto& sh = outShears[i];
        auto& r = outRotMats[i];
        auto& t = outTrans[i];
        auto& rm = restMats[i];
        auto& m = outMats[i];
        outLogMats.push_back(compose(sc, sh, r, t) * rm);
        outMats.push_back(m * rm);
    }
    setAllData(dataBlock, outMats, outLogMats, outAAs);
    return MStatus::kSuccess;
}
