#ifndef PXSB_FUSION_DF_COORDINATE_TRANSFORM_H
#define PXSB_FUSION_DF_COORDINATE_TRANSFORM_H
#include "DF_MATRIX3.h"
// #define Re 6378.160    //地球赤道半径
#define Re 6378140.0 //    //地球赤道半径2002.4.7     WGS84
// #define e2 0.993305458  //=B^2/Re^2 B--极半径
#define e2 0.993305524945510 // e2 0.993305458  //=B^2/Re^2 B--极半径

#ifndef M_PI
#define M_PI (double)3.14159265359
#endif
#ifndef M_DR
#define M_DR (double)0.01745329251994 // 度--->弧度 量化因子
#endif
#ifndef M_RD
#define M_RD (double)57.29577951 // 弧度--->度 量化因子
#endif
extern double delta_cta;
//---WGS84地心直角坐标--->BJ54地心直角坐标系  七参数  add by gjh
extern double delta_m; // 尺度参数
extern double cita_x;  // 三个方向的旋转角参数
extern double cita_y;
extern double cita_z;
extern double delta_x; // 三个方向的平移参数
extern double delta_y;
extern double delta_z;

class CCoordinate
{
public:
    double radar_jd, radar_wd, radar_h;
    double radar_jd_other, radar_wd_other, radar_h_other; // 它  雷达344a 坐标经纬度 add by gjh
    CCoordinate()
    {
        radar_jd = 119.2231543472;
        radar_wd = 31.955472556;
        radar_h = 69.680;
    };
    CCoordinate(double j, double w, double h)
    {
        radar_jd = j;
        radar_wd = w;
        radar_h = h;
    };
    void set_position_origin(double jd, double wd, double h)
    {
        radar_jd = jd;
        radar_wd = wd;
        radar_h = h;
    };
    void set_position_other(double jd, double wd, double h)
    {
        radar_jd_other = jd;
        radar_wd_other = wd;
        radar_h_other = h;
    };
    void rae_xyz(double r, double a, double e, double &xx, double &yy, double &zz);    // 单位：米、度---->米
    void xyz_rae(double xx, double yy, double zz, double &r, double &a, double &e);    // 单位：米---->米、度
    void xyz_XYZ(double x, double y, double z, double &XX, double &YY, double &ZZ);    // 单位：米---->米
    void XYZ_xyz(double XX, double YY, double ZZ, double &xx, double &yy, double &zz); // 单位：米---->米
    void XYZ_jwh(double XX, double YY, double ZZ, double &jd, double &wd, double &h);  // 单位：米---->度、米
    void jwh_XYZ(double jd, double wd, double h, double &XX, double &YY, double &ZZ);  // 单位：度、米---->米

    void rae_jwh(double r, double a, double e, double &jd, double &wd, double &h); // 单位：米、度---->度、米
    void jwh_rae(double jd, double wd, double h, double &r, double &a, double &e); // 单位：度、米---->米、度

    void jwh_XYZ_new(double jd, double wd, double h, double &XX, double &YY, double &ZZ);
    void jwh_rae_new(double jd, double wd, double h, double &r, double &a, double &e);

    void XYZ_xyz54(double XX, double YY, double ZZ, double &xx, double &yy, double &zz);
    void jwh_XYZ_3D(double jd, double wd, double h, double &XX, double &YY, double &ZZ);
    void jwh_rae_3D(double jd, double wd, double h, double &r, double &a, double &e);

    // 2003.10.08 3q new
    void jwh_rae_3q(double jd, double wd, double h, double &r1, double &a1, double &e1); // 单位：度、米---->米、度

    //----它  球体坐标到本  球体坐标 add by gjh
    void rae_rae_3q(double r, double a, double e, double &r1, double &a1, double &e1);
    //----它  球体坐标到本  球体坐标 add by gjh
    void rae_rae(double r, double a, double e, double &r1, double &a1, double &e1);

    //--速度坐标转换函数---20110329-------------------------------------------------------------------------------------------------

    //----速度坐标变换 rae->xyz add by gjh 20110329
    void vrae_vxyz(double r, double a, double e, double vr, double va, double ve, double &vx, double &vy, double &vz);
    //----速度坐标变换  xyz->rae  add by gjh 20110329
    void vxyz_vrae(double x, double y, double z, double vx, double vy, double vz, double &vr, double &va, double &ve);
    //----速度坐标变换  xyz->XYZ  add by gjh 20110329
    void vxyz_vXYZ(double vX0, double vY0, double vZ0, double vx, double vy, double vz, double &vX, double &vY, double &vZ);
    //--------空间直角坐标到表面直角坐标
    void vXYZ_vxyz(double vX0, double vY0, double vZ0, double vX, double vY, double vZ, double &vx, double &vy, double &vz);
    //----速度及位置坐标变换 rae->xyz add by gjh 20110329
    // 本  平台目标rae vr va ve  本  平台速度 vX0 vY0 vZ0       它  平台速度 vX1 vY1 vZ1  输出在它  平台下的 r1 a1 e1 vr1 va1 ve1
    void vrae_vrae(double r, double a, double e, double vr, double va, double ve, double vX0, double vY0, double vZ0, double vX1, double vY1, double vZ1, double &r1, double &a1, double &e1, double &vr1, double &va1, double &ve1);

    //----协方差、速度及位置坐标变换 rae->rae add by gjh 20110329
    // 本  平台目标rae vr va ve sigma_r  sigma_a sigma_e   本  平台速度 vX0 vY0 vZ0       它  平台速度 vX1 vY1 vZ1    输出在它  平台下的 r1 a1 e1 vr1 va1 ve1 sig_r1,sig_a1,sig_e1
    void Prae_Prae(double r, double a, double e, double vr, double va, double ve, double sig_r, double sig_a, double sig_e, double vX0, double vY0, double vZ0, double vX1, double vY1, double vZ1,
                   double &r1, double &a1, double &e1, double &vr1, double &va1, double &ve1, double &sig_r1, double &sig_a1, double &sig_e1);
};

#endif // PXSB_FUSION_DF_COORDINATE_TRANSFORM_H
