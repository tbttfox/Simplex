#include "blendPose.h"

#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <maya/MQuaternion.h>

#include <cmath>
#include <map>
#include <tuple>
#include <unordered_map>

#include "Eigen/Dense"
#include "Eigen/Eigenvalues"
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

MTypeId blendPose::id(0x00122717);
MString blendPose::typeName("blendPose");

// Attribute list
MObject blendPose::aOutputPose;
MObject blendPose::aOutputRawPose;
MObject blendPose::aOriginalPose;
MObject blendPose::aInputTarget;
MObject blendPose::aInputTargetPose;
MObject blendPose::aInputTargetLevel;
MObject blendPose::aWeight;

void* blendPose::creator() { return new blendPose(); }

MStatus blendPose::initialize() {
    MFnNumericAttribute nAttr;
    MFnMatrixAttribute mAttr;
    MFnCompoundAttribute cAttr;
    MStatus status;

    aOutputPose = mAttr.create("outputPose", "out", MFnMatrixAttribute::kDouble, &status);
    CHECKSTATCREATE(status);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setWritable(false);
    mAttr.setArray(true);
    status = addAttribute(aOutputPose);
    CHECKSTATADDATTR(status);

    aOutputRawPose = mAttr.create("outputRawPose", "raw", MFnMatrixAttribute::kDouble, &status);
    CHECKSTATCREATE(status);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setWritable(false);
    mAttr.setArray(true);
    status = addAttribute(aOutputRawPose);
    CHECKSTATADDATTR(status);

    aOriginalPose = mAttr.create("originalPose", "og", MFnMatrixAttribute::kDouble, &status);
    CHECKSTATCREATE(status);
    mAttr.setArray(true);
    mAttr.setStorable(true);
    status = addAttribute(aOriginalPose);
    CHECKSTATADDATTR(status);

    aInputTargetPose = mAttr.create("inputTargetPose", "itp", MFnMatrixAttribute::kDouble, &status);
    CHECKSTATCREATE(status);
    mAttr.setArray(true);
    mAttr.setStorable(true);

    aInputTargetLevel = nAttr.create("inputTargetLevel", "itl", MFnNumericData::kInt, 0, &status);
    nAttr.setMin(0);
    CHECKSTATCREATE(status);
    nAttr.setStorable(true);

    aInputTarget = cAttr.create("inputTarget", "it", &status);
    CHECKSTATCREATE(status);
    cAttr.addChild(aInputTargetPose);
    cAttr.addChild(aInputTargetLevel);
    cAttr.setArray(true);
    status = addAttribute(aInputTarget);
    CHECKSTATADDATTR(status);

    aWeight = nAttr.create("weight", "w", MFnNumericData::kFloat, 0.0, &status);
    nAttr.setSoftMin(0.0);
    nAttr.setSoftMax(1.0);
    nAttr.setArray(true);
    nAttr.setKeyable(true);
    nAttr.setStorable(true);
    status = addAttribute(aWeight);
    CHECKSTATADDATTR(status);

    status = attributeAffects(aOriginalPose, aOutputPose);
    CHECKSTATAFFECTS(status);
    status = attributeAffects(aInputTargetPose, aOutputPose);
    CHECKSTATAFFECTS(status);
    status = attributeAffects(aInputTarget, aOutputPose);
    CHECKSTATAFFECTS(status);
    status = attributeAffects(aWeight, aOutputPose);
    CHECKSTATAFFECTS(status);
    status = attributeAffects(aOriginalPose, aOutputRawPose);
    CHECKSTATAFFECTS(status);
    status = attributeAffects(aInputTargetPose, aOutputRawPose);
    CHECKSTATAFFECTS(status);
    status = attributeAffects(aInputTarget, aOutputRawPose);
    CHECKSTATAFFECTS(status);
    status = attributeAffects(aWeight, aOutputRawPose);
    CHECKSTATAFFECTS(status);

    return MS::kSuccess;
}

void blendPose::getAllData(
    MDataBlock& dataBlock,
    std::unordered_map<int, std::tuple<int, float, std::vector<MMatrix>>>& targets,
    std::vector<MMatrix>& restPose
) {
    getDenseArrayHandleData<MMatrix>(dataBlock, aOriginalPose, restPose);

    std::vector<float> weightPose;
    getDenseArrayHandleData<float>(dataBlock, aWeight, weightPose);

    auto targetArrayHandle = dataBlock.inputArrayValue(aInputTarget);
    for (auto [poseIdx, targetIndexHandle] : MArrayDataHandleRange(targetArrayHandle)) {
        if (poseIdx >= weightPose.size()) {
            continue;
        }
        if (std::fabs(weightPose[poseIdx]) < 1.0e-12) {
            continue;
        }
        int poseLevel = targetIndexHandle.child(aInputTargetLevel).asInt();
        MArrayDataHandle poseArrayHandle = targetIndexHandle.child(aInputTargetPose);
        std::vector<MMatrix> targetPose;
        getDenseArrayHandleData<MMatrix>(poseArrayHandle, targetPose);
        targets[poseIdx] = std::make_tuple(poseLevel, weightPose[poseIdx], targetPose);
    }
}

void blendPose::setAllData(
    const MPlug& plug, MDataBlock& dataBlock, std::vector<MMatrix>& rawMats,
    std::vector<MMatrix>& quatMats
) {
    auto outputRawArrayHandle = dataBlock.outputArrayValue(aOutputRawPose);
    auto outputRawBuilder = outputRawArrayHandle.builder();
    for (UINT idx = 0; idx < rawMats.size(); ++idx) {
        auto outputRawHandle = outputRawBuilder.addElement(idx);
        outputRawHandle.set(rawMats[idx]);
    }

    auto outputArrayHandle = dataBlock.outputArrayValue(aOutputPose);
    auto outputBuilder = outputArrayHandle.builder();
    for (UINT idx = 0; idx < quatMats.size(); ++idx) {
        auto outputQuatHandle = outputBuilder.addElement(idx);
        outputQuatHandle.set(quatMats[idx]);
    }
    outputRawArrayHandle.setAllClean();
    outputArrayHandle.setAllClean();
    dataBlock.setClean(plug);
}

void blendPose::computeRawMats(
    const std::unordered_map<int, std::tuple<int, float, std::vector<MMatrix>>>& targets,
    const std::vector<MMatrix>& restPose, std::vector<MMatrix>& rawMats
) {
    rawMats.clear();
    rawMats.reserve(restPose.size());
    static MMatrix zeromat = MMatrix() - MMatrix();
    for (size_t matIdx = 0; matIdx < restPose.size(); ++matIdx) {
        rawMats.push_back(zeromat);
    }

    for (auto& [poseIdx, poseData] : targets) {
        const auto& [_level, weight, pose] = poseData;
        if (fabs(weight) < 1.0e-12) {
            continue;
        }
        for (size_t matIdx = 0; matIdx < pose.size(); ++matIdx) {
            rawMats[matIdx] += weight * (pose[matIdx] - restPose[matIdx]);
        }
    }

    for (size_t matIdx = 0; matIdx < restPose.size(); ++matIdx) {
        rawMats[matIdx] += restPose[matIdx];
    }
}

Eigen::Matrix4f outerq(MQuaternion& q) {
    if (q.w < 0) {
        q.negateIt();
    }
    Eigen::Matrix4f ret;
    for (size_t i = 0; i < 4; ++i) {
        ret.col(i)[i] = q[i] * q[i];
        for (size_t j = i + 1; j < 4; ++j) {
            float t = q[i] * q[j];
            ret.col(i)[j] = t;
            ret.col(j)[i] = t;
        }
    }
    return ret;
}

void blendPose::computeQuatMats(
    const std::unordered_map<int, std::tuple<int, float, std::vector<MMatrix>>>& targets,
    const std::vector<MMatrix>& restPose, std::vector<MMatrix>& quatMats
) {
    MQuaternion qi;

    std::unordered_map<int, std::vector<Eigen::Matrix4f>> qmats;
    std::unordered_map<int, float> levelCounts;

    for (auto& [poseIdx, poseData] : targets) {
        const auto& [level, weight, pose] = poseData;
        if (fabs(weight) < 1.0e-12) {
            continue;
        }
        auto& qmatlevel = qmats[level];
        if (qmatlevel.size() == 0) {
            qmatlevel.reserve(restPose.size());
            for (size_t i = 0; i < restPose.size(); ++i) {
                qmatlevel.push_back(Eigen::Matrix4f::Zero());
            }
        }

        for (size_t matIdx = 0; matIdx < pose.size(); ++matIdx) {
            MQuaternion matquat, restquat, slerped;
            matquat = pose[matIdx];
            restquat = restPose[matIdx];
            slerped = slerp(qi, (matquat * restquat.inverse()), weight);
            qmatlevel[matIdx] += outerq(slerped);
            levelCounts[level] += 1.0f;
        }
    }

    std::map<int, std::vector<MQuaternion>> eigs;
    for (auto& [level, qmatpose] : qmats) {
        float count = levelCounts[level];
        std::vector<MQuaternion> leveleigs = eigs[level];
        for (size_t matIdx = 0; matIdx < qmatpose.size(); ++matIdx) {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> es(qmatpose[matIdx]);
            auto egval = es.eigenvalues();
            int maxidx = 0;
            float maxval = fabs(egval[0]);
            for (int i = 1; i < 4; ++i) {
                if (fabs(egval[i]) > maxval) {
                    maxidx = i;
                    maxval = fabs(egval[i]);
                }
            }
            auto evec = es.eigenvectors().col(maxidx);
            MQuaternion q(evec[0], evec[1], evec[2], evec[3]);
            q = slerp(qi, q, count);
            if (q.w < 0) {
                q.negateIt();
            }
            leveleigs.push_back(q.normal());
        }
    }

    quatMats = restPose;
    for (auto& [level, qpose] : eigs) {
        for (size_t i = 0; i < qpose.size(); ++i) {
            quatMats[i] = qpose[i].asMatrix() * quatMats[i];
        }
    }
}

MStatus blendPose::compute(const MPlug& plug, MDataBlock& dataBlock) {
    MStatus status;

    std::unordered_map<int, std::tuple<int, float, std::vector<MMatrix>>> targets;
    std::vector<MMatrix> restPose;
    getAllData(dataBlock, targets, restPose);

    if (targets.size() == 0) {
        setAllData(plug, dataBlock, restPose, restPose);
        return status;
    }

    std::vector<MMatrix> rawMats, quatMats;
    computeRawMats(targets, restPose, rawMats);
    computeQuatMats(targets, restPose, quatMats);

    for (size_t idx = 0; idx < rawMats.size(); ++idx) {
        auto r = rawMats[idx];
        auto q = quatMats[idx];
        q[3][0] = r[3][0];
        q[3][1] = r[3][1];
        q[3][2] = r[3][2];
    }

    setAllData(plug, dataBlock, rawMats, quatMats);
    return status;
}
