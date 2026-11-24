/**
 * @file glibc_compat.c
 * @brief 为旧版 glibc 提供缺失符号的兼容实现
 *
 * 现代 glibc 新增的符号:
 *   GLIBC_2.36: arc4random    — 密码学安全的随机数 (来自 OpenBSD)
 *   GLIBC_2.38: __isoc23_strtoul — C23 版本的 strtoul
 *
 * 编译时将此文件链接进 dds_receiver 即可在旧系统上运行。
 * 用法:
 *   g++ ... glibc_compat.c ... -o dds_receiver
 */

#define _GNU_SOURCE
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/random.h>

/* ── arc4random (GLIBC_2.36) ────────────────────────────────────
 * 用 /dev/urandom 或 getrandom(2) 实现 */
uint32_t arc4random(void) {
    uint32_t val;
    /* 优先用 getrandom (Linux 3.17+, glibc 2.25+) */
    if (getrandom(&val, sizeof(val), GRND_NONBLOCK) == sizeof(val))
        return val;

    /* 回退到 /dev/urandom */
    int fd = open("/dev/urandom", O_RDONLY | O_CLOEXEC);
    if (fd >= 0) {
        ssize_t n = read(fd, &val, sizeof(val));
        close(fd);
        if (n == sizeof(val))
            return val;
    }

    /* 最后回退: 不安全的 rand()，但总比崩溃强 */
    return (uint32_t)(rand() ^ (rand() << 16));
}

void arc4random_buf(void *buf, size_t n) {
    uint8_t *p = (uint8_t *)buf;
    /* 尝试 getentropy (glibc 2.25+) */
    if (getrandom(p, n, GRND_NONBLOCK) == (ssize_t)n)
        return;

    /* 回退: 逐块用 arc4random 填充 */
    while (n >= 4) {
        uint32_t v = arc4random();
        memcpy(p, &v, 4);
        p += 4; n -= 4;
    }
    if (n > 0) {
        uint32_t v = arc4random();
        memcpy(p, &v, n);
    }
}

uint32_t arc4random_uniform(uint32_t upper_bound) {
    if (upper_bound < 2) return 0;
    uint32_t r, min = -upper_bound % upper_bound;
    do {
        r = arc4random();
    } while (r < min);
    return r % upper_bound;
}

/* ── __isoc23_strtoul (GLIBC_2.38) ──────────────────────────────
 * C23 版本的 strtoul 行为与 C99 版本基本相同，回退到标准 strtoul */
unsigned long __isoc23_strtoul(const char *__restrict nptr,
                                char **__restrict endptr, int base) {
    return strtoul(nptr, endptr, base);
}
