/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Intel IPU7 processing system (PSYS) device
 *
 * Thin wrapper around the PSYS character device of Intel's (out-of-tree, GPL)
 * intel-ipu7-psys driver: graph open, buffer registration and synchronous task
 * execution.
 */

#include "psys.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <libcamera/base/log.h>

#include "ipu7-psys.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPU7)

PSysDevice::~PSysDevice()
{
	close();
}

int PSysDevice::open(const std::string &path)
{
	int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		int ret = -errno;
		LOG(IPU7, Debug) << "Failed to open " << path << ": " << strerror(-ret);
		return ret;
	}
	fd_ = UniqueFD(fd);
	return 0;
}

void PSysDevice::close()
{
	closeGraph();
	for (Buffer &b : owned_)
		free(b.mem);
	owned_.clear();
	fd_.reset();
}

int PSysDevice::openGraph(const std::vector<uint8_t> &info, const std::vector<uint8_t> &nodes)
{
	if (info.size() != sizeof(ipu_psys_graph_info))
		return -EINVAL;

	nodes_ = nodes;
	ipu_psys_graph_info g;
	memcpy(&g, info.data(), sizeof(g));
	g.nodes = reinterpret_cast<graph_node *>(nodes_.data());
	g.graph_id = 0xff;

	if (ioctl(fd_.get(), IPU_IOC_GRAPH_OPEN, &g) < 0) {
		int ret = -errno;
		LOG(IPU7, Error) << "GRAPH_OPEN failed: " << strerror(-ret);
		return ret;
	}
	graphId_ = g.graph_id;
	return 0;
}

void PSysDevice::closeGraph()
{
	if (graphId_ == 0xff || !fd_.isValid())
		return;
	int id = graphId_;
	ioctl(fd_.get(), IPU_IOC_GRAPH_CLOSE, &id);
	graphId_ = 0xff;
}

int PSysDevice::allocate(size_t len, Buffer *buf)
{
	void *mem = nullptr;
	if (posix_memalign(&mem, 4096, (len + 4095) & ~size_t(4095)))
		return -ENOMEM;
	memset(mem, 0, len);

	ipu_psys_buffer pb = {};
	pb.len = len;
	pb.base.userptr = mem;
	pb.flags = IPU_BUFFER_FLAG_USERPTR;
	if (ioctl(fd_.get(), IPU_IOC_GETBUF, &pb) < 0 ||
	    ioctl(fd_.get(), IPU_IOC_MAPBUF, (void *)(intptr_t)pb.base.fd) < 0) {
		int ret = -errno;
		LOG(IPU7, Error) << "PSYS buffer registration failed: " << strerror(-ret);
		free(mem);
		return ret;
	}

	buf->mem = mem;
	buf->len = len;
	buf->fd = pb.base.fd;
	owned_.push_back(*buf);
	return 0;
}

int PSysDevice::import(int fd, size_t len, Buffer *buf)
{
	if (ioctl(fd_.get(), IPU_IOC_MAPBUF, (void *)(intptr_t)fd) < 0) {
		int ret = -errno;
		LOG(IPU7, Error) << "MAPBUF of dma-buf " << fd << " failed: " << strerror(-ret);
		return ret;
	}
	buf->mem = nullptr;
	buf->len = len;
	buf->fd = fd;
	return 0;
}

void PSysDevice::unimport(const Buffer &buf)
{
	ioctl(fd_.get(), IPU_IOC_UNMAPBUF, (void *)(intptr_t)buf.fd);
}

int PSysDevice::run(uint8_t node, uint8_t frame, const std::vector<TermBuffer> &terms,
		    int timeoutMs)
{
	ipu_psys_term_buffers tb[MAX_GRAPH_TERMINALS] = {};
	unsigned int n = 0;
	for (const auto &[term, b] : terms) {
		if (n >= MAX_GRAPH_TERMINALS)
			return -EINVAL;
		tb[n].term_id = term;
		tb[n].term_buf.len = b->len;
		tb[n].term_buf.base.fd = b->fd;
		tb[n].term_buf.bytes_used = b->len;
		tb[n].term_buf.flags = IPU_BUFFER_FLAG_DMA_HANDLE | IPU_BUFFER_FLAG_NO_FLUSH;
		n++;
	}

	ipu_psys_task_request t = {};
	t.graph_id = graphId_;
	t.node_ctx_id = node;
	t.frame_id = frame;
	t.term_buf_count = n;
	t.task_buffers = tb;
	if (ioctl(fd_.get(), IPU_IOC_TASK_REQUEST, &t) < 0) {
		int ret = -errno;
		LOG(IPU7, Error) << "TASK_REQUEST node " << int(node) << " failed: " << strerror(-ret);
		return ret;
	}

	pollfd pfd = { fd_.get(), POLLIN, 0 };
	int ret = poll(&pfd, 1, timeoutMs);
	if (ret <= 0) {
		LOG(IPU7, Error) << "PSYS node " << int(node) << " frame " << int(frame) << " timed out";
		return -ETIMEDOUT;
	}

	ipu_psys_event ev = {};
	if (ioctl(fd_.get(), IPU_IOC_DQEVENT, &ev) < 0)
		return -errno;
	if (ev.error) {
		LOG(IPU7, Warning) << "PSYS node " << int(node) << " frame " << int(frame)
				   << " error " << ev.error;
		return -EIO;
	}
	return 0;
}

} /* namespace libcamera */
