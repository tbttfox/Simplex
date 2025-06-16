#include "matutils.h"

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
