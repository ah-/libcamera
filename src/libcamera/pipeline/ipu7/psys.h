/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Intel IPU7 processing system (PSYS) device
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>
#include <utility>
#include <vector>

#include <libcamera/base/unique_fd.h>

namespace libcamera {

class PSysDevice
{
public:
	/* A buffer registered with the PSYS (owned memory or an imported dma-buf) */
	struct Buffer {
		void *mem = nullptr;	/* CPU mapping (owned buffers only) */
		size_t len = 0;
		int fd = -1;		/* dma-buf fd known to the PSYS */
	};

	using TermBuffer = std::pair<uint8_t, const Buffer *>;

	PSysDevice() = default;
	~PSysDevice();

	int open(const std::string &path);
	void close();

	int openGraph(const std::vector<uint8_t> &info, const std::vector<uint8_t> &nodes);
	void closeGraph();

	int allocate(size_t len, Buffer *buf);
	int import(int fd, size_t len, Buffer *buf);
	void unimport(const Buffer &buf);

	struct Event {
		uint8_t node;
		uint8_t frame;
		uint32_t error;
	};

	int fd() const { return fd_.get(); }
	bool isOpen() const { return fd_.isValid(); }

	/* Queue a task; completion is reported through an event (see dequeue()). */
	int submit(uint8_t node, uint8_t frame, const std::vector<TermBuffer> &terms);
	/* Dequeue one completion event (call when fd() is readable). */
	int dequeue(Event *event);
	/* Wait up to timeoutMs for one event. */
	int wait(Event *event, int timeoutMs);

private:
	UniqueFD fd_;
	uint8_t graphId_ = 0xff;
	std::vector<uint8_t> nodes_;
	std::vector<Buffer> owned_;
};

} /* namespace libcamera */
