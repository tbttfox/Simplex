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

MMatrix set3x3(
    double a, double b, double c, double d, double e, double f, double g, double h, double i
) {
    MMatrix ret;
    ret(0, 0) = a;
    ret(0, 1) = b;
    ret(0, 2) = c;

    ret(1, 0) = d;
    ret(1, 1) = e;
    ret(1, 2) = f;

    ret(2, 0) = g;
    ret(2, 1) = h;
    ret(2, 2) = i;
    return ret;
}

MVector make_nice_vec(double a, double b, double c, double t) {
    return MVector(
        fabs(a - t) > 1e-12 ? a : t, fabs(b - t) > 1e-12 ? b : t, fabs(c - t) > 1e-12 ? c : t
    );
}

std::tuple<MVector, MVector, MMatrix, MVector> decompose(
    const MMatrix& mm, MVector& scale, MVector& shear, MMatrix& rot, MVector& tran
) {
    tran = MVector(mm(2, 0), mm(2, 1), mm(2, 2));

    // Now what is left is a 3X3 matrix which is considered to be a combination
    // of rotation, shearing and scaling.
    // We make use of the fact that for a rotation R the following is always valid:
    // R^T = R^{-1}

    // We now consider the remaining 3X3 matrix M to be the combination of
    // a rotation R and a shear/scale matrix S:
    // M = S * R
    // M * M^T = (S * R) * (S * R)^T
    // M * M^T = S * R * R^T * S^T
    // M * M^T = S * R * R^{-1} * S^T  // because R^T = R^{-1}
    // M * M^T = S * S^T
    //
    // This means that we can derive S from the last equation. For this we
    // assume that S is a triangle matrix in the form
    //     | a  0  0 |
    // S = | d  b  0 |
    //     | e  f  c |
    //
    //
    // Multiplying S * S^T results in
    //           | a2     a*d        a*e      |
    // S * S^T = | a*d    b2+d2      b*f+d*e  | = M * M^T
    //           | a*e    b*f+d*e    c2+e2+f2 |

    // Resolving this leads to the formulas below
    // The squares allow to take plus or minus values for the
    // scaling coefficients on the diagonal. For symmetry we
    // use either always + or always -, although switching just one
    // of them would suffice. This selects 1 of the 8 possible solutions
    // and takes care that any mirroring operation is put into the
    // scaling, so all other transformations in the chain have det=1.

    MMatrix mmt = mm * mm.transpose();
    double minus = mmt.det3x3() > 0 ? 1.0 : -1.0;
    double a = sqrt(mmt(0, 0)) * minus;
    double d = mmt(0, 1) / a;
    double e = mmt(0, 2) / a;

    double b = sqrt(mmt(1, 1) - d * d) * minus;
    double f = (mmt(1, 2) - d * e) / b;

    double c = sqrt(mmt(2, 2) - e * e - f * f);

    MMatrix scsh = set3x3(a, 0, 0, d, b, 0, e, f, c);

    // Having shear-and-scale we can calculate the rotation from
    // S * R = M  <=>
    // S^{-1} * S * R = S^{-1} * M
    // R = S^{-1} * M
    rot = scsh.inverse() * mm;
    rot(2, 0) = 0.0;
    rot(2, 1) = 0.0;
    rot(2, 2) = 0.0;

    scale = make_nice_vec(a, b, c, 1.0);
    //           | 1  0  0 |
    // Shear =>  | i  1  0 |
    //           | j  k  1 |
    //                   | a    0    0 |       | a  0  0 |
    // Scale * Shear =>  | b*i  b    0 | = S = | d  b  0 |
    //                   | c*j  c*k  c |       | e  f  c |

    shear = make_nice_vec(d / b, e / c, f / c, 0.0);

    return {scale, shear, rot, tran};
}

MMatrix compose(
    const MVector& scale, const MVector& shear, const MMatrix& rot, const MVector& tran
) {
    MMatrix(sc);
    sc(0, 0) = scale[0];
    sc(1, 1) = scale[1];
    sc(2, 2) = scale[2];

    MMatrix(sh);
    sh(1, 0) = shear[0];
    sh(2, 0) = shear[1];
    sh(2, 1) = shear[2];

    MMatrix ret = sc * sh * rot;
    ret(3, 0) = tran[0];
    ret(3, 1) = tran[1];
    ret(3, 2) = tran[2];
    return ret;
}

MVector logm(const MMatrix& m) {
    MMatrix S = (m - m.transpose()) * 0.5;
    MVector ret(S(1, 0), S(2, 0), S(2, 1));
    float theta = acos((m(0, 0) + m(1, 1) + m(2, 2) - 1) / 2);
    if (fabs(theta) > 1e-12) {
        ret *= theta / sin(theta);
    }
    return ret;
}

MMatrix expm(const MVector& v) {
    double theta = sqrt(v * v);

    if (theta < 1e-12) {
        return MMatrix();
    }

    MMatrix A;
    A *= 0;
    A(1, 0) = v[0];
    A(2, 0) = v[1];
    A(2, 1) = v[2];
    A(0, 1) = -v[0];
    A(0, 2) = -v[1];
    A(1, 2) = -v[2];

    MMatrix A2 = A * A;
    double st = sin(theta) / theta;
    double ct = (1 - cos(theta)) / (theta * theta);
    MMatrix ret = MMatrix() + st * A + ct * A2;
    ret(3, 3) = 1.0;
    return ret;
}

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
