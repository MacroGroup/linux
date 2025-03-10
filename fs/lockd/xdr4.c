// SPDX-License-Identifier: GPL-2.0
/*
 * linux/fs/lockd/xdr4.c
 *
 * XDR support for lockd and the lock client.
 *
 * Copyright (C) 1995, 1996 Olaf Kirch <okir@monad.swb.de>
 * Copyright (C) 1999, Trond Myklebust <trond.myklebust@fys.uio.no>
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/nfs.h>

#include <linux/sunrpc/xdr.h>
#include <linux/sunrpc/clnt.h>
#include <linux/sunrpc/svc.h>
#include <linux/sunrpc/stats.h>
#include <linux/lockd/lockd.h>

#include "svcxdr.h"

static inline s64
loff_t_to_s64(loff_t offset)
{
	s64 res;
	if (offset > NLM4_OFFSET_MAX)
		res = NLM4_OFFSET_MAX;
	else if (offset < -NLM4_OFFSET_MAX)
		res = -NLM4_OFFSET_MAX;
	else
		res = offset;
	return res;
}

void nlm4svc_set_file_lock_range(struct file_lock *fl, u64 off, u64 len)
{
	s64 end = off + len - 1;

	fl->fl_start = off;
	if (len == 0 || end < 0)
		fl->fl_end = OFFSET_MAX;
	else
		fl->fl_end = end;
}

/*
 * NLM file handles are defined by specification to be a variable-length
 * XDR opaque no longer than 1024 bytes. However, this implementation
 * limits their length to the size of an NFSv3 file handle.
 */
static bool
svcxdr_decode_fhandle(struct xdr_stream *xdr, struct nfs_fh *fh)
{
	__be32 *p;
	u32 len;

	if (xdr_stream_decode_u32(xdr, &len) < 0)
		return false;
	if (len > NFS_MAXFHSIZE)
		return false;

	p = xdr_inline_decode(xdr, len);
	if (!p)
		return false;
	fh->size = len;
	memcpy(fh->data, p, len);
	memset(fh->data + len, 0, sizeof(fh->data) - len);

	return true;
}

static bool
svcxdr_decode_lock(struct xdr_stream *xdr, struct nlm_lock *lock)
{
	struct file_lock *fl = &lock->fl;

	if (!svcxdr_decode_string(xdr, &lock->caller, &lock->len))
		return false;
	if (!svcxdr_decode_fhandle(xdr, &lock->fh))
		return false;
	if (!svcxdr_decode_owner(xdr, &lock->oh))
		return false;
	if (xdr_stream_decode_u32(xdr, &lock->svid) < 0)
		return false;
	if (xdr_stream_decode_u64(xdr, &lock->lock_start) < 0)
		return false;
	if (xdr_stream_decode_u64(xdr, &lock->lock_len) < 0)
		return false;

	locks_init_lock(fl);
	fl->c.flc_type  = F_RDLCK;
	nlm4svc_set_file_lock_range(fl, lock->lock_start, lock->lock_len);
	return true;
}

static bool
svcxdr_encode_holder(struct xdr_stream *xdr, const struct nlm_lock *lock)
{
	const struct file_lock *fl = &lock->fl;
	s64 start, len;

	/* exclusive */
	if (xdr_stream_encode_bool(xdr, fl->c.flc_type != F_RDLCK) < 0)
		return false;
	if (xdr_stream_encode_u32(xdr, lock->svid) < 0)
		return false;
	if (!svcxdr_encode_owner(xdr, &lock->oh))
		return false;
	start = loff_t_to_s64(fl->fl_start);
	if (fl->fl_end == OFFSET_MAX)
		len = 0;
	else
		len = loff_t_to_s64(fl->fl_end - fl->fl_start + 1);
	if (xdr_stream_encode_u64(xdr, start) < 0)
		return false;
	if (xdr_stream_encode_u64(xdr, len) < 0)
		return false;

	return true;
}

static bool
svcxdr_encode_testrply(struct xdr_stream *xdr, const struct nlm_res *resp)
{
	if (!svcxdr_encode_stats(xdr, resp->status))
		return false;
	switch (resp->status) {
	case nlm_lck_denied:
		if (!svcxdr_encode_holder(xdr, &resp->lock))
			return false;
	}

	return true;
}


/*
 * Decode Call arguments
 */

bool
nlm4svc_decode_void(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	return true;
}

bool
nlm4svc_decode_testargs(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	struct nlm_args *argp = rqstp->rq_argp;
	u32 exclusive;

	if (!svcxdr_decode_cookie(xdr, &argp->cookie))
		return false;
	if (xdr_stream_decode_bool(xdr, &exclusive) < 0)
		return false;
	if (!svcxdr_decode_lock(xdr, &argp->lock))
		return false;
	if (exclusive)
		argp->lock.fl.c.flc_type = F_WRLCK;

	return true;
}

bool
nlm4svc_decode_lockargs(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	struct nlm_args *argp = rqstp->rq_argp;
	u32 exclusive;

	if (!svcxdr_decode_cookie(xdr, &argp->cookie))
		return false;
	if (xdr_stream_decode_bool(xdr, &argp->block) < 0)
		return false;
	if (xdr_stream_decode_bool(xdr, &exclusive) < 0)
		return false;
	if (!svcxdr_decode_lock(xdr, &argp->lock))
		return false;
	if (exclusive)
		argp->lock.fl.c.flc_type = F_WRLCK;
	if (xdr_stream_decode_bool(xdr, &argp->reclaim) < 0)
		return false;
	if (xdr_stream_decode_u32(xdr, &argp->state) < 0)
		return false;
	argp->monitor = 1;		/* monitor client by default */

	return true;
}

bool
nlm4svc_decode_cancargs(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	struct nlm_args *argp = rqstp->rq_argp;
	u32 exclusive;

	if (!svcxdr_decode_cookie(xdr, &argp->cookie))
		return false;
	if (xdr_stream_decode_bool(xdr, &argp->block) < 0)
		return false;
	if (xdr_stream_decode_bool(xdr, &exclusive) < 0)
		return false;
	if (!svcxdr_decode_lock(xdr, &argp->lock))
		return false;
	if (exclusive)
		argp->lock.fl.c.flc_type = F_WRLCK;

	return true;
}

bool
nlm4svc_decode_unlockargs(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	struct nlm_args *argp = rqstp->rq_argp;

	if (!svcxdr_decode_cookie(xdr, &argp->cookie))
		return false;
	if (!svcxdr_decode_lock(xdr, &argp->lock))
		return false;
	argp->lock.fl.c.flc_type = F_UNLCK;

	return true;
}

bool
nlm4svc_decode_res(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	struct nlm_res *resp = rqstp->rq_argp;

	if (!svcxdr_decode_cookie(xdr, &resp->cookie))
		return false;
	if (!svcxdr_decode_stats(xdr, &resp->status))
		return false;

	return true;
}

bool
nlm4svc_decode_reboot(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	struct nlm_reboot *argp = rqstp->rq_argp;
	__be32 *p;
	u32 len;

	if (xdr_stream_decode_u32(xdr, &len) < 0)
		return false;
	if (len > SM_MAXSTRLEN)
		return false;
	p = xdr_inline_decode(xdr, len);
	if (!p)
		return false;
	argp->len = len;
	argp->mon = (char *)p;
	if (xdr_stream_decode_u32(xdr, &argp->state) < 0)
		return false;
	p = xdr_inline_decode(xdr, SM_PRIV_SIZE);
	if (!p)
		return false;
	memcpy(&argp->priv.data, p, sizeof(argp->priv.data));

	return true;
}

bool
nlm4svc_decode_shareargs(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	struct nlm_args *argp = rqstp->rq_argp;
	struct nlm_lock	*lock = &argp->lock;

	locks_init_lock(&lock->fl);
	lock->svid = ~(u32)0;

	if (!svcxdr_decode_cookie(xdr, &argp->cookie))
		return false;
	if (!svcxdr_decode_string(xdr, &lock->caller, &lock->len))
		return false;
	if (!svcxdr_decode_fhandle(xdr, &lock->fh))
		return false;
	if (!svcxdr_decode_owner(xdr, &lock->oh))
		return false;
	/* XXX: Range checks are missing in the original code */
	if (xdr_stream_decode_u32(xdr, &argp->fsm_mode) < 0)
		return false;
	if (xdr_stream_decode_u32(xdr, &argp->fsm_access) < 0)
		return false;

	return true;
}

bool
nlm4svc_decode_notify(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	struct nlm_args *argp = rqstp->rq_argp;
	struct nlm_lock	*lock = &argp->lock;

	if (!svcxdr_decode_string(xdr, &lock->caller, &lock->len))
		return false;
	if (xdr_stream_decode_u32(xdr, &argp->state) < 0)
		return false;

	return true;
}


/*
 * Encode Reply results
 */

bool
nlm4svc_encode_void(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	return true;
}

bool
nlm4svc_encode_testres(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	struct nlm_res *resp = rqstp->rq_resp;

	return svcxdr_encode_cookie(xdr, &resp->cookie) &&
		svcxdr_encode_testrply(xdr, resp);
}

bool
nlm4svc_encode_res(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	struct nlm_res *resp = rqstp->rq_resp;

	return svcxdr_encode_cookie(xdr, &resp->cookie) &&
		svcxdr_encode_stats(xdr, resp->status);
}

bool
nlm4svc_encode_shareres(struct svc_rqst *rqstp, struct xdr_stream *xdr)
{
	struct nlm_res *resp = rqstp->rq_resp;

	if (!svcxdr_encode_cookie(xdr, &resp->cookie))
		return false;
	if (!svcxdr_encode_stats(xdr, resp->status))
		return false;
	/* sequence */
	if (xdr_stream_encode_u32(xdr, 0) < 0)
		return false;

	return true;
}
