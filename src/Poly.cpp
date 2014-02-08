#include "Poly.hpp"

Poly3 Poly3New(double x, double y, double z, double unit)
{
    Poly3 p;

    for (int i = 0; i < num_poly3_coefs; i++) p.v[i] = 0.0;

    p.v[coef_x] = x;
    p.v[coef_y] = y;
    p.v[coef_z] = z;
    p.v[coef_1] = unit;

    return p;
}

Poly3 Poly3Add(Poly3 a, Poly3 b)
{
    Poly3 r;

    for (int i = 0; i < num_poly3_coefs; i++) r.v[i] = a.v[i] + b.v[i];

    return r;
}

Poly3 Poly3Add3(Poly3 a, Poly3 b, Poly3 c)
{
    Poly3 r;

    for (int i = 0; i < num_poly3_coefs; i++) r.v[i] = a.v[i] + b.v[i] + c.v[i];

    return r;
}

Poly3 Poly3Sub(Poly3 a, Poly3 b)
{
    Poly3 r;

    for (int i = 0; i < num_poly3_coefs; i++) r.v[i] = a.v[i] - b.v[i];

    return r;
}

Poly3 Poly3Mult(Poly3 a, Poly3 b)
{
    Poly3 r;

    r.v[coef_1]   = a.v[coef_1]   * b.v[coef_1];
    r.v[coef_x]   = a.v[coef_x]   * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_x];
    r.v[coef_y]   = a.v[coef_y]   * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_y];
    r.v[coef_z]   = a.v[coef_z]   * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_z];

    r.v[coef_xy]  = a.v[coef_xy]  * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_xy]  + a.v[coef_x] * b.v[coef_y] + a.v[coef_y] * b.v[coef_x];
    r.v[coef_xz]  = a.v[coef_xz]  * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_xz]  + a.v[coef_x] * b.v[coef_z] + a.v[coef_z] * b.v[coef_x];
    r.v[coef_yz]  = a.v[coef_yz]  * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_yz]  + a.v[coef_y] * b.v[coef_z] + a.v[coef_z] * b.v[coef_y];

    r.v[coef_xx]  = a.v[coef_xx]  * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_xx]  + a.v[coef_x] * b.v[coef_x];
    r.v[coef_yy]  = a.v[coef_yy]  * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_yy]  + a.v[coef_y] * b.v[coef_y];
    r.v[coef_zz]  = a.v[coef_zz]  * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_zz]  + a.v[coef_z] * b.v[coef_z];

    r.v[coef_xxy] = a.v[coef_xxy] * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_xxy] +
                    a.v[coef_xx]  * b.v[coef_y]  + a.v[coef_y]  * b.v[coef_xx]  +
                    a.v[coef_xy]  * b.v[coef_x]  + a.v[coef_x]  * b.v[coef_xy];

    r.v[coef_xxz] = a.v[coef_xxz] * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_xxz] +
                    a.v[coef_xx]  * b.v[coef_z]  + a.v[coef_z]  * b.v[coef_xx]  +
                    a.v[coef_xz]  * b.v[coef_x]  + a.v[coef_x]  * b.v[coef_xz];

    r.v[coef_xyy] = a.v[coef_xyy] * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_xyy] +
                    a.v[coef_x]   * b.v[coef_yy] + a.v[coef_yy] * b.v[coef_x]   +
                    a.v[coef_xy]  * b.v[coef_y]  + a.v[coef_y]  * b.v[coef_xy];

    r.v[coef_yyz] = a.v[coef_yyz] * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_yyz] +
                    a.v[coef_yy]  * b.v[coef_z]  + a.v[coef_z]  * b.v[coef_yy]  +
                    a.v[coef_yz]  * b.v[coef_y]  + a.v[coef_y]  * b.v[coef_yz];

    r.v[coef_xzz] = a.v[coef_xzz] * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_xzz] +
                    a.v[coef_x]   * b.v[coef_zz] + a.v[coef_zz] * b.v[coef_x]   +
                    a.v[coef_xz]  * b.v[coef_z]  + a.v[coef_z]  * b.v[coef_xz];

    r.v[coef_yzz] = a.v[coef_yzz] * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_yzz] +
                    a.v[coef_y]   * b.v[coef_zz] + a.v[coef_zz] * b.v[coef_y]   +
                    a.v[coef_yz]  * b.v[coef_z]  + a.v[coef_z]  * b.v[coef_yz];

    r.v[coef_xyz] = a.v[coef_xyz] * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_xyz] +
                    a.v[coef_xy]  * b.v[coef_z]  + a.v[coef_z]  * b.v[coef_xy]  +
                    a.v[coef_xz]  * b.v[coef_y]  + a.v[coef_y]  * b.v[coef_xz]  +
                    a.v[coef_yz]  * b.v[coef_x]  + a.v[coef_x]  * b.v[coef_yz];

    r.v[coef_xxx] = a.v[coef_xxx] * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_xxx] +
                    a.v[coef_xx]  * b.v[coef_x]  + a.v[coef_x]  * b.v[coef_xx];

    r.v[coef_yyy] = a.v[coef_yyy] * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_yyy] +
                    a.v[coef_yy]  * b.v[coef_y]  + a.v[coef_y]  * b.v[coef_yy];

    r.v[coef_zzz] = a.v[coef_zzz] * b.v[coef_1]  + a.v[coef_1]  * b.v[coef_zzz] +
                    a.v[coef_zz]  * b.v[coef_z]  + a.v[coef_z]  * b.v[coef_zz];

    return r;
}

Poly3 Poly3Mult11(Poly3 a, Poly3 b)
{
    Poly3 r;

    for (int i = 0; i < num_poly3_coefs; i++) r.v[i] = 0.0;

    r.v[coef_1]  = a.v[coef_1] * b.v[coef_1];
    r.v[coef_x]  = a.v[coef_x] * b.v[coef_1] + a.v[coef_1] * b.v[coef_x];
    r.v[coef_y]  = a.v[coef_y] * b.v[coef_1] + a.v[coef_1] * b.v[coef_y];
    r.v[coef_z]  = a.v[coef_z] * b.v[coef_1] + a.v[coef_1] * b.v[coef_z];

    r.v[coef_xy] = a.v[coef_x] * b.v[coef_y] + a.v[coef_y] * b.v[coef_x];
    r.v[coef_xz] = a.v[coef_x] * b.v[coef_z] + a.v[coef_z] * b.v[coef_x];
    r.v[coef_yz] = a.v[coef_y] * b.v[coef_z] + a.v[coef_z] * b.v[coef_y];

    r.v[coef_xx] = a.v[coef_x] * b.v[coef_x];
    r.v[coef_yy] = a.v[coef_y] * b.v[coef_y];
    r.v[coef_zz] = a.v[coef_z] * b.v[coef_z];

    return r;
}

Poly3 Poly3Mult21(Poly3 a, Poly3 b)
{
    Poly3 r;

    r.v[coef_1]   = a.v[coef_1]  * b.v[coef_1];
    r.v[coef_x]   = a.v[coef_x]  * b.v[coef_1] + a.v[coef_1]  * b.v[coef_x];
    r.v[coef_y]   = a.v[coef_y]  * b.v[coef_1] + a.v[coef_1]  * b.v[coef_y];
    r.v[coef_z]   = a.v[coef_z]  * b.v[coef_1] + a.v[coef_1]  * b.v[coef_z];

    r.v[coef_xy]  = a.v[coef_xy] * b.v[coef_1] + a.v[coef_x]  * b.v[coef_y] + a.v[coef_y]  * b.v[coef_x];
    r.v[coef_xz]  = a.v[coef_xz] * b.v[coef_1] + a.v[coef_x]  * b.v[coef_z] + a.v[coef_z]  * b.v[coef_x];
    r.v[coef_yz]  = a.v[coef_yz] * b.v[coef_1] + a.v[coef_y]  * b.v[coef_z] + a.v[coef_z]  * b.v[coef_y];

    r.v[coef_xx]  = a.v[coef_xx] * b.v[coef_1] + a.v[coef_x]  * b.v[coef_x];
    r.v[coef_yy]  = a.v[coef_yy] * b.v[coef_1] + a.v[coef_y]  * b.v[coef_y];
    r.v[coef_zz]  = a.v[coef_zz] * b.v[coef_1] + a.v[coef_z]  * b.v[coef_z];

    r.v[coef_xxy] = a.v[coef_xx] * b.v[coef_y] + a.v[coef_xy] * b.v[coef_x];
    r.v[coef_xxz] = a.v[coef_xx] * b.v[coef_z] + a.v[coef_xz] * b.v[coef_x];
    r.v[coef_xyy] = a.v[coef_yy] * b.v[coef_x] + a.v[coef_xy] * b.v[coef_y];
    r.v[coef_yyz] = a.v[coef_yy] * b.v[coef_z] + a.v[coef_yz] * b.v[coef_y];
    r.v[coef_xzz] = a.v[coef_zz] * b.v[coef_x] + a.v[coef_xz] * b.v[coef_z];
    r.v[coef_yzz] = a.v[coef_zz] * b.v[coef_y] + a.v[coef_yz] * b.v[coef_z];
    r.v[coef_xyz] = a.v[coef_xy] * b.v[coef_z] + a.v[coef_xz] * b.v[coef_y] + a.v[coef_yz] * b.v[coef_x];

    r.v[coef_xxx] = a.v[coef_xx] * b.v[coef_x];
    r.v[coef_yyy] = a.v[coef_yy] * b.v[coef_y];
    r.v[coef_zzz] = a.v[coef_zz] * b.v[coef_z];

    return r;
}

Poly3 Poly3Scale(Poly3 a, double scale)
{
    Poly3 r;

    for (int i = 0; i < num_poly3_coefs; i++) r.v[i] = scale * a.v[i];

    return r;
}
