#pragma once

#include <maya/MMatrix.h>
#include <maya/MVector.h>

#include <tuple>

MMatrix set3x3(
    double a, double b, double c, double d, double e, double f, double g, double h, double i
);

MVector make_nice_vec(double a, double b, double c, double t);

std::tuple<MVector, MVector, MMatrix, MVector> decompose(
    const MMatrix& mm, MVector& scale, MVector& shear, MMatrix& rot, MVector& tran
);

MMatrix compose(
    const MVector& scale, const MVector& shear, const MMatrix& rot, const MVector& tran
);

MVector logm(const MMatrix& m);

MMatrix expm(const MVector& v);
