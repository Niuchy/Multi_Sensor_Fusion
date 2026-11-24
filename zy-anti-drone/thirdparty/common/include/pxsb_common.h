/**
 * @file
 * header: common PXSB functions and macros
 */

#ifndef __PXSB_COMMON_H__
#define __PXSB_COMMON_H__

#include <sys/time.h>
#ifdef ANDROID
#include <android/log.h>
#endif

#if defined WIN32 && defined PXSB_LIB
/** Functions exported from DLL's */
#define EXPORT __declspec(dllexport)
#elif defined WIN32 && !defined PXSB_LIB
/** Functions imported from DLL's */
#define EXPORT __declspec(dllimport)
#else
/** Not used in UNIX-like systems, but required for compatible with WIN32 libraries */
#define EXPORT
#endif

EXPORT long long tvdiff_us(struct timeval *tv1, struct timeval *tv2);
EXPORT long long pxsb_usleep(long long interval, struct timeval *ptv);
EXPORT int pxsb_log(const char *fmt, ...);
EXPORT int pxsb_error(const char *fmt, ...);
EXPORT int pxsb_malloc(int size, void **ptr);
EXPORT int pxsb_malloc(int size, void **ptr, int *alignment);
EXPORT int pxsb_alignment(void *ptr, int alignto);
EXPORT void pxsb_openlog();
EXPORT void pxsb_closelog();
EXPORT void pxsb_setlog(const char *name);
// aggregated output feature
EXPORT void pxsb_aggregated_reset();
EXPORT void pxsb_aggregated_print(int key, int limit, int value);
//
EXPORT long pxsb_atoi(const char *str);
EXPORT void pxsb_backtrace();

// print detail info for debug.

// 日志等级
enum LOG_LEVEL
{
    INFO = 1,
    DEBUG = 2,
    WARNING = 3,
    ERROR = 4,
};

EXPORT int pxsb_log(const LOG_LEVEL log_level, const char *fmt, ...);

/**
 * @brief 加载日志配置文件
 */
EXPORT int pxsb_load_log_conf();

/**
 * @brief 关闭日志
 */
EXPORT void pxsb_close_log_conf();

#endif
