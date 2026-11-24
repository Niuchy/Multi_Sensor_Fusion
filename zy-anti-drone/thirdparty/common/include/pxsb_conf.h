/**
 * @file
 * GamingAnywhere configuration file loader: header files
 */

#ifndef __PXSB_CONF_H__
#define __PXSB_CONF_H__

#include "pxsb_common.h"

// laod config from file
EXPORT int pxsb_conf_load(const char *filename);
EXPORT int pxsb_url_parse(const char *url);
EXPORT void pxsb_conf_clear();

// operations with key/value pair
EXPORT char *pxsb_conf_readv(const char *key, char *store, int slen);
EXPORT int pxsb_conf_readint(const char *key);
EXPORT double pxsb_conf_readdouble(const char *key);
EXPORT int pxsb_conf_readbool(const char *key, int defval);
EXPORT int pxsb_conf_boolval(const char *ptr, int defval);
EXPORT int pxsb_conf_readints(const char *key, int *vals, int n);
EXPORT int pxsb_conf_writev(const char *key, const char *value);
EXPORT void pxsb_conf_erase(const char *key);

// operations with map
EXPORT int pxsb_conf_ismap(const char *key);
EXPORT int pxsb_conf_haskey(const char *mapname, const char *key);
EXPORT int pxsb_conf_mapsize(const char *mapname);
EXPORT char *pxsb_conf_mapreadv(const char *mapname, const char *key, char *store, int slen);
EXPORT int pxsb_conf_mapreadint(const char *mapname, const char *key);
EXPORT int pxsb_conf_mapreadints(const char *mapname, const char *key, int *vals, int n);
EXPORT double pxsb_conf_mapreaddouble(const char *mapname, const char *key);
EXPORT int pxsb_conf_mapreadbool(const char *mapname, const char *key, int defval);
EXPORT int pxsb_conf_mapwritev(const char *mapname, const char *key, const char *value);
EXPORT void pxsb_conf_maperase(const char *mapname, const char *key);
//
EXPORT void pxsb_conf_mapreset(const char *mapname);
EXPORT char *pxsb_conf_mapkey(const char *mapname, char *keystore, int klen);
EXPORT char *pxsb_conf_mapvalue(const char *mapname, char *valstore, int vlen);
EXPORT char *pxsb_conf_mapnextkey(const char *mapname, char *keystore, int klen);

// operations with whole map
EXPORT void pxsb_conf_reset();
EXPORT const char *pxsb_conf_key();
EXPORT const char *pxsb_conf_nextkey();

#endif /* __PXSB_CONF_H__ */
