/*
 * Copyright (C) 2011  Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#define pr_fmt(fmt) "llcp: %s: " fmt, __func__

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nfc.h>

#include "../nfc.h"
#include "llcp.h"

static struct proto llcp_sock_proto = {
	.name     = "NFC_LLCP",
	.owner    = THIS_MODULE,
	.obj_size = sizeof(struct nfc_llcp_sock),
};

static int llcp_sock_bind(struct socket *sock, struct sockaddr *addr, int alen)
{
	struct sock *sk = sock->sk;
	struct nfc_llcp_sock *llcp_sock = nfc_llcp_sock(sk);
	struct nfc_llcp_local *local;
	struct nfc_dev *dev;
	struct sockaddr_nfc_llcp llcp_addr;
	int len, ret = 0;

	pr_debug("sk %p addr %p family %d\n", sk, addr, addr->sa_family);

	if (!addr || addr->sa_family != AF_NFC)
		return -EINVAL;

	memset(&llcp_addr, 0, sizeof(llcp_addr));
	len = min_t(unsigned int, sizeof(llcp_addr), alen);
	memcpy(&llcp_addr, addr, len);

	/* This is going to be a listening socket, dsap must be 0 */
	if (llcp_addr.dsap != 0)
		return -EINVAL;

	lock_sock(sk);

	if (sk->sk_state != LLCP_CLOSED) {
		ret = -EBADFD;
		goto error;
	}

	dev = nfc_get_device(llcp_addr.dev_idx);
	if (dev == NULL) {
		ret = -ENODEV;
		goto error;
	}

	local = nfc_llcp_find_local(dev);
	if (local == NULL) {
		ret = -ENODEV;
		goto put_dev;
	}

	llcp_sock->dev = dev;
	llcp_sock->local = local;
	llcp_sock->nfc_protocol = llcp_addr.nfc_protocol;
	llcp_sock->service_name_len = min_t(unsigned int,
			llcp_addr.service_name_len, NFC_LLCP_MAX_SERVICE_NAME);
	llcp_sock->service_name = kmemdup(llcp_addr.service_name,
				llcp_sock->service_name_len, GFP_KERNEL);

	llcp_sock->ssap = nfc_llcp_get_sdp_ssap(local, llcp_sock);
	if (llcp_sock->ssap == LLCP_MAX_SAP)
		goto put_dev;

	local->sockets[llcp_sock->ssap] = llcp_sock;

	pr_debug("Socket bound to SAP %d\n", llcp_sock->ssap);

	sk->sk_state = LLCP_BOUND;

put_dev:
	nfc_put_device(dev);

error:
	release_sock(sk);
	return ret;
}

static int llcp_sock_listen(struct socket *sock, int backlog)
{
	struct sock *sk = sock->sk;
	int ret = 0;

	pr_debug("sk %p backlog %d\n", sk, backlog);

	lock_sock(sk);

	if ((sock->type != SOCK_SEQPACKET && sock->type != SOCK_STREAM)
			|| sk->sk_state != LLCP_BOUND) {
		ret = -EBADFD;
		goto error;
	}

	sk->sk_max_ack_backlog = backlog;
	sk->sk_ack_backlog = 0;

	pr_debug("Socket listening\n");
	sk->sk_state = LLCP_LISTEN;

error:
	release_sock(sk);

	return ret;
}

void nfc_llcp_accept_unlink(struct sock *sk)
{
	struct nfc_llcp_sock *llcp_sock = nfc_llcp_sock(sk);

	pr_debug("state %d\n", sk->sk_state);

	list_del_init(&llcp_sock->accept_queue);
	sk_acceptq_removed(llcp_sock->parent);
	llcp_sock->parent = NULL;

	sock_put(sk);
}

void nfc_llcp_accept_enqueue(struct sock *parent, struct sock *sk)
{
	struct nfc_llcp_sock *llcp_sock = nfc_llcp_sock(sk);
	struct nfc_llcp_sock *llcp_sock_parent = nfc_llcp_sock(parent);

	/* Lock will be free from unlink */
	sock_hold(sk);

	list_add_tail(&llcp_sock->accept_queue,
			&llcp_sock_parent->accept_queue);
	llcp_sock->parent = parent;
	sk_acceptq_added(parent);
}

struct sock *nfc_llcp_accept_dequeue(struct sock *parent,
					struct socket *newsock)
{
	struct nfc_llcp_sock *lsk, *n, *llcp_parent;
	struct sock *sk;

	llcp_parent = nfc_llcp_sock(parent);

	list_for_each_entry_safe(lsk, n, &llcp_parent->accept_queue,
							accept_queue) {
		sk = &lsk->sk;
		lock_sock(sk);

		if (sk->sk_state == LLCP_CLOSED) {
			release_sock(sk);
			nfc_llcp_accept_unlink(sk);
			continue;
		}

		if (sk->sk_state == LLCP_CONNECTED || !newsock) {
			nfc_llcp_accept_unlink(sk);
			if (newsock)
				sock_graft(sk, newsock);

			release_sock(sk);

			pr_debug("Returning sk state %d\n", sk->sk_state);

			return sk;
		}

		release_sock(sk);
	}

	return NULL;
}

static int llcp_sock_accept(struct socket *sock, struct socket *newsock,
								int flags)
{
	DECLARE_WAITQUEUE(wait, current);
	struct sock *sk = sock->sk, *new_sk;
	long timeo;
	int ret = 0;

	pr_debug("parent %p\n", sk);

	lock_sock_nested(sk, SINGLE_DEPTH_NESTING);

	if (sk->sk_state != LLCP_LISTEN) {
		ret = -EBADFD;
		goto error;
	}

	timeo = sock_rcvtimeo(sk, flags & O_NONBLOCK);

	/* Wait for an incoming connection. */
	add_wait_queue_exclusive(sk_sleep(sk), &wait);
	while (!(new_sk = nfc_llcp_accept_dequeue(sk, newsock))) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (!timeo) {
			ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = sock_intr_errno(timeo);
			break;
		}

		release_sock(sk);
		timeo = schedule_timeout(timeo);
		lock_sock_nested(sk, SINGLE_DEPTH_NESTING);
	}
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(sk_sleep(sk), &wait);

	if (ret)
		goto error;

	newsock->state = SS_CONNECTED;

	pr_debug("new socket %p\n", new_sk);

error:
	release_sock(sk);

	return ret;
}

static int llcp_sock_getname(struct socket *sock, struct sockaddr *addr,
			     int *len, int peer)
{
	struct sockaddr_nfc_llcp *llcp_addr = (struct sockaddr_nfc_llcp *) addr;
	struct sock *sk = sock->sk;
	struct nfc_llcp_sock *llcp_sock = nfc_llcp_sock(sk);

	pr_debug("%p\n", sk);

	addr->sa_family = AF_NFC;
	*len = sizeof(struct sockaddr_nfc_llcp);

	llcp_addr->dev_idx = llcp_sock->dev->idx;
	llcp_addr->dsap = llcp_sock->dsap;
	llcp_addr->ssap = llcp_sock->ssap;
	llcp_addr->service_name_len = llcp_sock->service_name_len;
	memcpy(llcp_addr->service_name, llcp_sock->service_name,
					llcp_addr->service_name_len);

	return 0;
}

static inline unsigned int llcp_accept_poll(struct sock *parent)
{
	struct nfc_llcp_sock *llcp_sock, *n, *parent_sock;
	struct sock *sk;

	parent_sock = nfc_llcp_sock(parent);

	list_for_each_entry_safe(llcp_sock, n, &parent_sock->accept_queue,
								accept_queue) {
		sk = &llcp_sock->sk;

		if (sk->sk_state == LLCP_CONNECTED)
			return POLLIN | POLLRDNORM;
	}

	return 0;
}

static unsigned int llcp_sock_poll(struct file *file, struct socket *sock,
							poll_table *wait)
{
	struct sock *sk = sock->sk;
	unsigned int mask = 0;

	pr_debug("%p\n", sk);

	sock_poll_wait(file, sk_sleep(sk), wait);

	if (sk->sk_state == LLCP_LISTEN)
		return llcp_accept_poll(sk);

	if (sk->sk_err || !skb_queue_empty(&sk->sk_error_queue))
		mask |= POLLERR;

	if (!skb_queue_empty(&sk->sk_receive_queue))
		mask |= POLLIN;

	if (sk->sk_state == LLCP_CLOSED)
		mask |= POLLHUP;

	return mask;
}

static int llcp_sock_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct nfc_llcp_local *local;
	struct nfc_llcp_sock *llcp_sock = nfc_llcp_sock(sk);

	if (!sk)
		return 0;

	pr_debug("%p\n", sk);

	local = llcp_sock->local;
	if (local == NULL)
		return -ENODEV;

	mutex_lock(&local->socket_lock);

	if (llcp_sock == local->sockets[llcp_sock->ssap]) {
		local->sockets[llcp_sock->ssap] = NULL;
	} else {
		struct nfc_llcp_sock *parent, *s, *n;

		parent = local->sockets[llcp_sock->ssap];

		list_for_each_entry_safe(s, n, &parent->list, list)
			if (llcp_sock == s) {
				list_del(&s->list);
				break;
			}

	}

	mutex_unlock(&local->socket_lock);

	lock_sock(sk);

	/* Send a DISC */
	if (sk->sk_state == LLCP_CONNECTED)
		nfc_llcp_disconnect(llcp_sock);

	if (sk->sk_state == LLCP_LISTEN) {
		struct nfc_llcp_sock *lsk, *n;
		struct sock *accept_sk;

		list_for_each_entry_safe(lsk, n, &llcp_sock->accept_queue,
								accept_queue) {
			accept_sk = &lsk->sk;
			lock_sock(accept_sk);

			nfc_llcp_disconnect(lsk);
			nfc_llcp_accept_unlink(accept_sk);

			release_sock(accept_sk);

			sock_set_flag(sk, SOCK_DEAD);
			sock_orphan(accept_sk);
			sock_put(accept_sk);
		}
	}

	/* Freeing the SAP */
	if ((sk->sk_state == LLCP_CONNECTED
			&& llcp_sock->ssap > LLCP_LOCAL_SAP_OFFSET) ||
	    sk->sk_state == LLCP_BOUND ||
	    sk->sk_state == LLCP_LISTEN)
		nfc_llcp_put_ssap(llcp_sock->local, llcp_sock->ssap);

	sock_set_flag(sk, SOCK_DEAD);

	release_sock(sk);

	sock_orphan(sk);
	sock_put(sk);

	return 0;
}

static int llcp_sock_connect(struct socket *sock, struct sockaddr *_addr,
							int len, int flags)
{
	struct sock *sk = sock->sk;
	struct nfc_llcp_sock *llcp_sock = nfc_llcp_sock(sk);
	struct sockaddr_nfc_llcp *addr = (struct sockaddr_nfc_llcp *)_addr;
	struct nfc_dev *dev;
	struct nfc_llcp_local *local;
	int ret = 0;

	pr_debug("sock %p sk %p flags 0x%x\n", sock, sk, flags);

	if (!addr || len < sizeof(struct sockaddr_nfc) ||
			addr->sa_family != AF_NFC) {
		pr_err("Invalid socket\n");
		return -EINVAL;
	}

	if (addr->service_name_len == 0 && addr->dsap == 0) {
		pr_err("Missing service name or dsap\n");
		return -EINVAL;
	}

	pr_debug("addr dev_idx=%u target_idx=%u protocol=%u\n", addr->dev_idx,
					addr->target_idx, addr->nfc_protocol);

	lock_sock(sk);

	if (sk->sk_state == LLCP_CONNECTED) {
		ret = -EISCONN;
		goto error;
	}

	dev = nfc_get_device(addr->dev_idx);
	if (dev == NULL) {
		ret = -ENODEV;
		goto error;
	}

	local = nfc_llcp_find_local(dev);
	if (local == NULL) {
		ret = -ENODEV;
		goto put_dev;
	}

	device_lock(&dev->dev);
	if (dev->dep_link_up == false) {
		ret = -ENOLINK;
		device_unlock(&dev->dev);
		goto put_dev;
	}
	device_unlock(&dev->dev);

	if (local->rf_mode == NFC_RF_INITIATOR &&
			addr->target_idx != local->target_idx) {
		ret = -ENOLINK;
		goto put_dev;
	}

	llcp_sock->dev = dev;
	llcp_sock->local = local;
	llcp_sock->ssap = nfc_llcp_get_local_ssap(local);
	if (llcp_sock->ssap == LLCP_SAP_MAX) {
		ret = -ENOMEM;
		goto put_dev;
	}
	if (addr->service_name_len == 0)
		llcp_sock->dsap = addr->dsap;
	else
		llcp_sock->dsap = LLCP_SAP_SDP;
	llcp_sock->nfc_protocol = addr->nfc_protocol;
	llcp_sock->service_name_len = min_t(unsigned int,
			addr->service_name_len, NFC_LLCP_MAX_SERVICE_NAME);
	llcp_sock->service_name = kmemdup(addr->service_name,
				 llcp_sock->service_name_len, GFP_KERNEL);

	local->sockets[llcp_sock->ssap] = llcp_sock;

	ret = nfc_llcp_send_connect(llcp_sock);
	if (ret)
		goto put_dev;

	sk->sk_state = LLCP_CONNECTED;

	release_sock(sk);
	return 0;

put_dev:
	nfc_put_device(dev);

error:
	release_sock(sk);
	return ret;
}

static int llcp_sock_sendmsg(struct kiocb *iocb, struct socket *sock,
				struct msghdr *msg, size_t len)
{
	struct sock *sk = sock->sk;
	struct nfc_llcp_sock *llcp_sock = nfc_llcp_sock(sk);
	int ret;

	pr_debug("sock %p sk %p", sock, sk);

	ret = sock_error(sk);
	if (ret)
		return ret;

	if (msg->msg_flags & MSG_OOB)
		return -EOPNOTSUPP;

	lock_sock(sk);

	if (sk->sk_state != LLCP_CONNECTED) {
		release_sock(sk);
		return -ENOTCONN;
	}

	release_sock(sk);

	return nfc_llcp_send_i_frame(llcp_sock, msg, len);
}

static int llcp_sock_recvmsg(struct kiocb *iocb, struct socket *sock,
			     struct msghdr *msg, size_t len, int flags)
{
	int noblock = flags & MSG_DONTWAIT;
	struct sock *sk = sock->sk;
	unsigned int copied, rlen;
	struct sk_buff *skb, *cskb;
	int err = 0;

	pr_debug("%p %zu\n", sk, len);

	lock_sock(sk);

	if (sk->sk_state == LLCP_CLOSED &&
			skb_queue_empty(&sk->sk_receive_queue)) {
		release_sock(sk);
		return 0;
	}

	release_sock(sk);

	if (flags & (MSG_OOB))
		return -EOPNOTSUPP;

	skb = skb_recv_datagram(sk, flags, noblock, &err);
	if (!skb) {
		pr_err("Recv datagram failed state %d %d %d",
				sk->sk_state, err, sock_error(sk));

		if (sk->sk_shutdown & RCV_SHUTDOWN)
			return 0;

		return err;
	}

	rlen   = skb->len;		/* real length of skb */
	copied = min_t(unsigned int, rlen, len);

	cskb = skb;
	if (memcpy_toiovec(msg->msg_iov, cskb->data, copied)) {
		if (!(flags & MSG_PEEK))
			skb_queue_head(&sk->sk_receive_queue, skb);
		return -EFAULT;
	}

	/* Mark read part of skb as used */
	if (!(flags & MSG_PEEK)) {

		/* SOCK_STREAM: re-queue skb if it contains unreceived data */
		if (sk->sk_type == SOCK_STREAM) {
			skb_pull(skb, copied);
			if (skb->len) {
				skb_queue_head(&sk->sk_receive_queue, skb);
				goto done;
			}
		}

		kfree_skb(skb);
	}

	/* XXX Queue backlogged skbs */

done:
	/* SOCK_SEQPACKET: return real length if MSG_TRUNC is set */
	if (sk->sk_type == SOCK_SEQPACKET && (flags & MSG_TRUNC))
		copied = rlen;

	return copied;
}

static const struct proto_ops llcp_sock_ops = {
	.family         = PF_NFC,
	.owner          = THIS_MODULE,
	.bind           = llcp_sock_bind,
	.connect        = llcp_sock_connect,
	.release        = llcp_sock_release,
	.socketpair     = sock_no_socketpair,
	.accept         = llcp_sock_accept,
	.getname        = llcp_sock_getname,
	.poll           = llcp_sock_poll,
	.ioctl          = sock_no_ioctl,
	.listen         = llcp_sock_listen,
	.shutdown       = sock_no_shutdown,
	.setsockopt     = sock_no_setsockopt,
	.getsockopt     = sock_no_getsockopt,
	.sendmsg        = llcp_sock_sendmsg,
	.recvmsg        = llcp_sock_recvmsg,
	.mmap           = sock_no_mmap,
};

static void llcp_sock_destruct(struct sock *sk)
{
	struct nfc_llcp_sock *llcp_sock = nfc_llcp_sock(sk);

	pr_debug("%p\n", sk);

	if (sk->sk_state == LLCP_CONNECTED)
		nfc_put_device(llcp_sock->dev);

	skb_queue_purge(&sk->sk_receive_queue);

	nfc_llcp_sock_free(llcp_sock);

	if (!sock_flag(sk, SOCK_DEAD)) {
		pr_err("Freeing alive NFC LLCP socket %p\n", sk);
		return;
	}
}

struct sock *nfc_llcp_sock_alloc(struct socket *sock, int type, gfp_t gfp)
{
	struct sock *sk;
	struct nfc_llcp_sock *llcp_sock;

	sk = sk_alloc(&init_net, PF_NFC, gfp, &llcp_sock_proto);
	if (!sk)
		return NULL;

	llcp_sock = nfc_llcp_sock(sk);

	sock_init_data(sock, sk);
	sk->sk_state = LLCP_CLOSED;
	sk->sk_protocol = NFC_SOCKPROTO_LLCP;
	sk->sk_type = type;
	sk->sk_destruct = llcp_sock_destruct;

	llcp_sock->ssap = 0;
	llcp_sock->dsap = LLCP_SAP_SDP;
	llcp_sock->send_n = llcp_sock->send_ack_n = 0;
	llcp_sock->recv_n = llcp_sock->recv_ack_n = 0;
	llcp_sock->remote_ready = 1;
	skb_queue_head_init(&llcp_sock->tx_queue);
	skb_queue_head_init(&llcp_sock->tx_pending_queue);
	skb_queue_head_init(&llcp_sock->tx_backlog_queue);
	INIT_LIST_HEAD(&llcp_sock->list);
	INIT_LIST_HEAD(&llcp_sock->accept_queue);

	if (sock != NULL)
		sock->state = SS_UNCONNECTED;

	return sk;
}

void nfc_llcp_sock_free(struct nfc_llcp_sock *sock)
{
	kfree(sock->service_name);

	skb_queue_purge(&sock->tx_queue);
	skb_queue_purge(&sock->tx_pending_queue);
	skb_queue_purge(&sock->tx_backlog_queue);

	list_del_init(&sock->accept_queue);

	sock->parent = NULL;
}

static int llcp_sock_create(struct net *net, struct socket *sock,
				const struct nfc_protocol *nfc_proto)
{
	struct sock *sk;

	pr_debug("%p\n", sock);

	if (sock->type != SOCK_STREAM && sock->type != SOCK_DGRAM)
		return -ESOCKTNOSUPPORT;

	sock->ops = &llcp_sock_ops;

	sk = nfc_llcp_sock_alloc(sock, sock->type, GFP_ATOMIC);
	if (sk == NULL)
		return -ENOMEM;

	return 0;
}

static const struct nfc_protocol llcp_nfc_proto = {
	.id	  = NFC_SOCKPROTO_LLCP,
	.proto    = &llcp_sock_proto,
	.owner    = THIS_MODULE,
	.create   = llcp_sock_create
};

int __init nfc_llcp_sock_init(void)
{
	return nfc_proto_register(&llcp_nfc_proto);
}

void nfc_llcp_sock_exit(void)
{
	nfc_proto_unregister(&llcp_nfc_proto);
}
