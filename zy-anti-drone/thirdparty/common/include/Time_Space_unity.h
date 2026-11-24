#ifndef PXSB_TIME_SPACE_UNITY_H
#define PXSB_TIME_SPACE_UNITY_H
#include "df_coordinate_transform.h"
#include "string"
#include <unordered_map>
#include <chrono>
#include <ctime>

using namespace std;

/*
 * 时间校准
 * 指挥端接收到数据后，校准用于处理数据的时间
 * 包括校准为本机时间、GPS天文时间、作战时间
 * 单位为ms,最小值0，代表1970.1.1 00：00：00.000
 * model = "BJ",代表与本机时间校准；"TW",代表与天文时间校准；"ZZ",代表与作战时间校准
 */
uint unityTime_Commander(uint time, string model);

/*
 * 时间校准
 * 非指挥端接收到数据后，校准自己执行任务的时间
 * 单位为ms,最小值0，代表1970.1.1 00：00：00.000
 * 其中time为指挥端发送报文的时间戳，time_mission为指挥端发送报文中的任务时间
 */
uint unityTime_Other(uint time, uint time_mission);

/*
 * 空间校准
 * 包括rae\jwh\xyz\XYZ两两坐标之间的相互转换
 */
//--------表面球体坐标到表面直角坐标
void rae_xyz_uinty(double r, double a, double e, double &xx, double &yy, double &zz); // 单位：米、度---->米
void xyz_rae_uinty(double xx, double yy, double zz, double &r, double &a, double &e); // 单位：米---->米、度

//--------表面直角坐标到空间直角坐标，jd\wd\h为要转换的xyz坐标系坐标原点
void xyz_XYZ_uinty(double jd, double wd, double h, double x, double y, double z, double &XX, double &YY, double &ZZ);    // 单位：米---->米
void XYZ_xyz_uinty(double jd, double wd, double h, double XX, double YY, double ZZ, double &xx, double &yy, double &zz); // 单位：米---->米

//----表面球体坐标到大地测量坐标
//----jd_other,wd_other,h_other：原rae相对坐标系的坐标原点
//----jd_origin,wd_origin,h_origin:要转换到的rae相对坐标系的坐标原点
void rae_jwh_uinty(double jd_other, double wd_other, double h_other, double r, double a, double e, double &jd, double &wd, double &h);       // 单位：米、度---->度、米
void jwh_rae_uinty(double jd_origin, double wd_origin, double h_origin, double jd, double wd, double h, double &r1, double &a1, double &e1); // 单位：度、米---->米、度

//----大地测量坐标到空间直角坐标
void XYZ_jwh_unity(double XX, double YY, double ZZ, double &jd, double &wd, double &h);
void jwh_XYZ_uinty(double jd, double wd, double h, double &XX, double &YY, double &ZZ);

//----它球体坐标到本球体坐标
//----jd_other,wd_other,h_other：原rae相对坐标系的坐标原点
//----jd_origin,wd_origin,h_origin:要转换到的rae相对坐标系的坐标原点
void rae_rae_uinty(double jd_other, double wd_other, double h_other, double jd_origin, double wd_origin, double h_origin, double r, double a, double e, double &r1, double &a1, double &e1);

#endif // PXSB_TIME_SPACE_UNITY_H
