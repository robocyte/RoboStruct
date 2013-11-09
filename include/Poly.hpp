#pragma once

enum Poly3Coefficients
{
    coef_1,
    coef_x, coef_y, coef_z, 
    coef_xy, coef_xz, coef_yz,
    coef_xx, coef_yy, coef_zz,
    coef_xxy, coef_xxz, coef_xyy, coef_yyz, coef_xzz, coef_yzz, coef_xyz,
    coef_xxx, coef_yyy, coef_zzz,
    num_poly3_coefs
};

struct Poly3
{
    double v[num_poly3_coefs];
};

Poly3 Poly3New(double x, double y, double z, double unit);
Poly3 Poly3Add(Poly3 a, Poly3 b);
Poly3 Poly3Add3(Poly3 a, Poly3 b, Poly3 c);
Poly3 Poly3Sub(Poly3 a, Poly3 b);
Poly3 Poly3Mult(Poly3 a, Poly3 b);
Poly3 Poly3Mult11(Poly3 a, Poly3 b);
Poly3 Poly3Mult21(Poly3 a, Poly3 b);
Poly3 Poly3Scale(Poly3 a, double scale);
