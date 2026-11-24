#ifndef __PXSB_APIPE_H__
#define __PXSB_APIPE_H__

/**
 * @file
 * apipe header: pipe for delivering discrete message
 */

#include <stdio.h>
#include <pthread.h>

#include "pxsb_common.h"

/**
 * structure for buffering a information
 */
typedef struct apipe_buffer_s
{
	void *pointer;				 /**< pointer to a information buffer. Aligned to 8-byte address: is equivalent to internal + offset */
	void *internal;				 /**< internal pointer to the allocated buffer space. Used with malloc() and free(). */
	int offset;					 /**< data pointer offset from internal */
	struct apipe_buffer_s *next; /**< pointer to the next apipe information buffer */
} apipe_buffer_t;

typedef struct apipe_s
{
	int channel_id; /**< channel id for the apipe */
	char *name;		/**< name of the apipe */
	//
	pthread_mutex_t cond_mutex; /**< pthread mutex for conditional signaling */
	pthread_cond_t cond;		/**< pthread condition */
	//
	pthread_mutex_t io_mutex; /**< apipe i/o pool operation mutex */
	apipe_buffer_t *in;		  /**< input pool: pointer to the first information buffer in input pool (free informations) */
	apipe_buffer_t *out;	  /**< output pool: pointer to the first information buffer in output pool (occupied informations) */
	apipe_buffer_t *out_tail; /**< output pool: pointer to the last information buffer in output pool (occupied informations) */
	int in_count;			  /**< number of unused information buffers */
	int out_count;			  /**< number of occupied informations */
} apipe_t;

EXPORT apipe_t *apipe_create(int id, const char *name, int ninfo, int maxinfosize);
EXPORT apipe_t *apipe_lookup(const char *name);
EXPORT int apipe_destroy(apipe_t *apipe);
EXPORT apipe_buffer_t *apipe_get(apipe_t *apipe);
EXPORT void apipe_put(apipe_t *apipe, apipe_buffer_t *buffer);
EXPORT apipe_buffer_t *apipe_load(apipe_t *apipe, const struct timespec *abstime);
EXPORT apipe_buffer_t *apipe_load_nowait(apipe_t *apipe);
EXPORT void apipe_store(apipe_t *apipe, apipe_buffer_t *buffer);

#endif /* __PXSB_APIPE_H__ */
