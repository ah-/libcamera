/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Pipeline handler for the Intel IPU7 hardware ISP (Panther Lake / IPU7.5)
 *
 * ISYS (capture) -> PSYS node 0 (LBFF: Bayer -> YUV) -> PSYS node 1 (BBPS: TNR, output) -> NV12.
 *
 * Requires Intel's intel-ipu7 / intel-ipu7-isys / intel-ipu7-psys kernel modules and the
 * ISP firmware. No Intel userspace code is used: the graph is a data file (LIBCAMERA_IPU7_DATA),
 * the per-frame parameters and 3A come from the ipu7 IPA module (tuning file imx471.yaml).
 * Parameter and statistics buffers are dma-bufs shared with the IPA. First version: one camera
 * (IMX471), one stream (NV12 1920x1080), processing runs synchronously on the pipeline thread.
 */

#include <algorithm>
#include <array>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <queue>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include <linux/media-bus-format.h>
#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/color_space.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/request.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include <libcamera/ipa/ipu7_ipa_interface.h>
#include <libcamera/ipa/ipu7_ipa_proxy.h>

#include "psys.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPU7)

namespace {

constexpr const char *kPsysPath = "/dev/ipu7-psys0";
constexpr const char *kCsi2Name = "Intel IPU7 CSI2 0";
constexpr const char *kCaptureName = "Intel IPU7 ISYS Capture 0";
constexpr const char *kDefaultDataDir = LIBCAMERA_DATA_DIR "/ipa/ipu7/imx471";

const Size kSensorSize(1928, 1088);
const Size kOutputSize(1920, 1080);
constexpr unsigned int kRawBufferCount = 4;
constexpr unsigned int kOutputBufferCount = 4;

/* PSYS buffer sizes for the recorded 1080p graph (IMX471) */
constexpr size_t kPsOutSize = 6226560;
constexpr size_t kMeOutSize = 398912;
constexpr size_t kOutSize = 3113280;
constexpr size_t kRefSize = 3113472;
constexpr size_t kHist13Size = 196352;
constexpr size_t kHist8Size = 130560;

std::string dataDir()
{
	const char *env = utils::secure_getenv("LIBCAMERA_IPU7_DATA");
	return env ? env : kDefaultDataDir;
}

bool readFile(const std::string &path, std::vector<uint8_t> *out)
{
	std::ifstream f(path, std::ios::binary);
	if (!f)
		return false;
	out->assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
	return true;
}

} /* namespace */

class IPU7CameraData : public Camera::Private
{
public:
	IPU7CameraData(PipelineHandler *pipe)
		: Camera::Private(pipe)
	{
	}

	int init(MediaDevice *media);
	int allocateSharedBuffers();
	int allocatePsysBuffers();
	void rawBufferReady(FrameBuffer *buffer);
	int process(FrameBuffer *raw, Request *request, FrameBuffer *out);
	void setSensorControls(ControlList &ctrls);

	Stream stream_;
	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<V4L2Subdevice> csi2_;
	std::unique_ptr<V4L2VideoDevice> capture_;

	std::vector<std::unique_ptr<FrameBuffer>> rawBuffers_;
	std::map<int, PSysDevice::Buffer> rawPsys_;	/* raw dma-buf fd -> registered buffer */
	std::queue<Request *> pendingRequests_;

	PSysDevice psys_;
	std::unique_ptr<ipa::ipu7::IPAProxyIPU7> ipa_;
	std::vector<uint8_t> graphInfo_, graphNodes_;

	/* Buffers shared with the IPA (dma-bufs): parameters written by it, statistics read by it */
	struct SharedBuffer {
		std::unique_ptr<FrameBuffer> fb;
		size_t size = 0;
		PSysDevice::Buffer psys;
	};
	std::map<ipa::ipu7::BufferId, SharedBuffer> shared_;
	std::unique_ptr<MappedFrameBuffer> outputMap_;
	const PSysDevice::Buffer *sb(ipa::ipu7::BufferId id) { return &shared_[id].psys; }

	/* PSYS-internal buffers */
	PSysDevice::Buffer stats11_, psOut_, meOut_;
	PSysDevice::Buffer ref_[2], hist13_[2], hist8_[2];

	unsigned int frame_ = 0;
	bool running_ = false;
};

class IPU7CameraConfiguration : public CameraConfiguration
{
public:
	Status validate() override;
};

class PipelineHandlerIPU7 : public PipelineHandler
{
public:
	PipelineHandlerIPU7(CameraManager *manager)
		: PipelineHandler(manager)
	{
	}

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								    Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;
	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;
	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;
	int queueRequestDevice(Camera *camera, Request *request) override;
	bool match(DeviceEnumerator *enumerator) override;

private:
	IPU7CameraData *cameraData(Camera *camera)
	{
		return static_cast<IPU7CameraData *>(camera->_d());
	}
};

/* -------------------------------------------------------------------------- */

CameraConfiguration::Status IPU7CameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	if (sensorConfig)
		return Invalid;

	if (orientation != Orientation::Rotate0) {
		orientation = Orientation::Rotate0;
		status = Adjusted;
	}

	if (config_.size() > 1) {
		config_.resize(1);
		status = Adjusted;
	}

	StreamConfiguration &cfg = config_[0];
	if (cfg.pixelFormat != formats::NV12 || cfg.size != kOutputSize) {
		cfg.pixelFormat = formats::NV12;
		cfg.size = kOutputSize;
		status = Adjusted;
	}
	cfg.stride = kOutputSize.width;
	cfg.frameSize = kOutputSize.width * kOutputSize.height * 3 / 2;
	cfg.colorSpace = ColorSpace::Rec709;
	if (cfg.bufferCount < 1)
		cfg.bufferCount = kOutputBufferCount;

	return status;
}

/* -------------------------------------------------------------------------- */

int IPU7CameraData::init(MediaDevice *media)
{
	MediaEntity *sensorEntity = nullptr;
	for (MediaEntity *entity : media->entities()) {
		if (entity->function() == MEDIA_ENT_F_CAM_SENSOR) {
			sensorEntity = entity;
			break;
		}
	}
	if (!sensorEntity) {
		LOG(IPU7, Debug) << "No sensor found";
		return -ENODEV;
	}

	sensor_ = CameraSensorFactoryBase::create(sensorEntity);
	if (!sensor_)
		return -ENODEV;

	csi2_ = V4L2Subdevice::fromEntityName(media, kCsi2Name);
	capture_ = V4L2VideoDevice::fromEntityName(media, kCaptureName);
	if (!csi2_ || !capture_)
		return -ENODEV;
	int ret = csi2_->open();
	if (ret)
		return ret;
	ret = capture_->open();
	if (ret)
		return ret;
	capture_->bufferReady.connect(this, &IPU7CameraData::rawBufferReady);

	const std::string dir = dataDir();
	if (!readFile(dir + "/graph/graph_info.bin", &graphInfo_) ||
	    !readFile(dir + "/graph/graph_nodes.bin", &graphNodes_)) {
		LOG(IPU7, Error) << "Missing graph files in " << dir;
		return -ENOENT;
	}
	ipa_ = pipe()->createIPA<ipa::ipu7::IPAProxyIPU7>(1, 1);
	if (!ipa_) {
		LOG(IPU7, Error) << "Failed to load the IPU7 IPA module";
		return -ENOENT;
	}
	const std::string tuningFile = ipa_->configurationFile(sensor_->model() + ".yaml");
	if (tuningFile.empty()) {
		LOG(IPU7, Error) << "No tuning file for " << sensor_->model();
		return -ENOENT;
	}
	ret = ipa_->init({ tuningFile, sensor_->model() });
	if (ret) {
		LOG(IPU7, Error) << "IPA init failed: " << ret;
		return ret;
	}
	ret = allocateSharedBuffers();
	if (ret)
		return ret;

	properties_ = sensor_->properties();
	controlInfo_ = ControlInfoMap({}, controls::controls);
	return 0;
}

int IPU7CameraData::allocateSharedBuffers()
{
	using ipa::ipu7::BufferId;
	static const std::pair<BufferId, size_t> list[] = {
		{ BufferId::LbffParams, 28288 }, { BufferId::LscTable, 17920 },
		{ BufferId::LbffProgram, 896 }, { BufferId::LbffSystem, 2688 },
		{ BufferId::BbpsParams, 2176 }, { BufferId::BbpsProgram, 64 },
		{ BufferId::BbpsSystem, 1728 }, { BufferId::BbpsFrame, 2624 },
		{ BufferId::StatsAe, 8192 }, { BufferId::StatsRgbs, 32640 }, { BufferId::StatsSat, 4096 },
		{ BufferId::Output, kOutSize },
	};

	DmaBufAllocator allocator(DmaBufAllocator::DmaBufAllocatorFlag::CmaHeap |
				  DmaBufAllocator::DmaBufAllocatorFlag::SystemHeap |
				  DmaBufAllocator::DmaBufAllocatorFlag::UDmaBuf);
	if (!allocator.isValid())
		return -ENODEV;

	std::vector<IPABuffer> ipaBuffers;
	for (const auto &[id, size] : list) {
		std::vector<std::unique_ptr<FrameBuffer>> fbs;
		const unsigned int alloc = (size + 4095) & ~4095u;
		if (allocator.exportBuffers(1, { alloc }, &fbs) != 1)
			return -ENOMEM;
		SharedBuffer &s = shared_[id];
		s.fb = std::move(fbs[0]);
		s.size = size;
		const auto &planes = s.fb->planes();
		ipaBuffers.emplace_back(static_cast<unsigned int>(id),
					std::vector<FrameBuffer::Plane>{ planes.begin(), planes.end() });
	}
	ipa_->mapBuffers(ipaBuffers);

	outputMap_ = std::make_unique<MappedFrameBuffer>(shared_[BufferId::Output].fb.get(),
							 MappedFrameBuffer::MapFlag::Read);
	return outputMap_->isValid() ? 0 : -ENOMEM;
}

int IPU7CameraData::allocatePsysBuffers()
{
	/* import the shared dma-bufs into the PSYS for this stream */
	for (auto &[id, s] : shared_) {
		int ret = psys_.import(s.fb->planes()[0].fd.get(), s.size, &s.psys);
		if (ret)
			return ret;
	}

	struct {
		PSysDevice::Buffer *buf;
		size_t size;
	} list[] = {
		{ &stats11_, 1856 }, { &psOut_, kPsOutSize }, { &meOut_, kMeOutSize },
		{ &ref_[0], kRefSize }, { &ref_[1], kRefSize },
		{ &hist13_[0], kHist13Size }, { &hist13_[1], kHist13Size },
		{ &hist8_[0], kHist8Size }, { &hist8_[1], kHist8Size },
	};
	for (auto &e : list) {
		int ret = psys_.allocate(e.size, e.buf);
		if (ret)
			return ret;
	}
	return 0;
}

void IPU7CameraData::setSensorControls(ControlList &ctrls)
{
	if (!ctrls.empty())
		sensor_->setControls(&ctrls);
}

int IPU7CameraData::process(FrameBuffer *raw, Request *request, FrameBuffer *out)
{
	using ipa::ipu7::BufferId;

	auto it = rawPsys_.find(raw->planes()[0].fd.get());
	if (it == rawPsys_.end())
		return -EINVAL;
	const PSysDevice::Buffer *rawBuf = &it->second;

	const uint32_t frame = frame_;
	const uint8_t fid = frame_ & 0xff;
	frame_++;	/* frame ids advance per submitted frame, even if it fails */

	ipa_->computeParams(frame);

	int ret = psys_.run(0, fid, {
		{ 12, sb(BufferId::StatsRgbs) }, { 11, &stats11_ }, { 10, sb(BufferId::StatsAe) },
		{ 8, sb(BufferId::LscTable) }, { 2, sb(BufferId::LbffSystem) }, { 1, sb(BufferId::LbffProgram) },
		{ 13, sb(BufferId::StatsSat) }, { 0, sb(BufferId::LbffParams) },
		{ 19, &psOut_ }, { 18, &meOut_ }, { 5, rawBuf },
	});
	if (ret)
		return ret;

	ControlList sensorCtrls;
	ControlList metadata;
	ipa_->processStats(frame, &sensorCtrls, &metadata);
	setSensorControls(sensorCtrls);

	const unsigned int cur = fid & 1 ? 0 : 1, prev = fid & 1;
	ret = psys_.run(1, fid, {
		{ 5, &hist13_[prev] }, { 7, &meOut_ }, { 9, &psOut_ }, { 14, sb(BufferId::Output) },
		{ 1, sb(BufferId::BbpsProgram) }, { 0, sb(BufferId::BbpsParams) }, { 13, &hist13_[cur] },
		{ 2, sb(BufferId::BbpsSystem) }, { 3, sb(BufferId::BbpsFrame) },
		{ 8, &hist8_[cur] }, { 11, &hist8_[cur] }, { 6, &hist8_[prev] },
		{ 12, &ref_[cur] }, { 10, &ref_[prev] },
	});
	if (ret)
		return ret;

	const unsigned int ySize = kOutputSize.width * kOutputSize.height;
	MappedFrameBuffer map(out, MappedFrameBuffer::MapFlag::Write);
	if (!map.isValid())
		return -ENOMEM;
	/* planes of an NV12 buffer from DmaBufAllocator are contiguous in one mapping */
	const uint8_t *src = outputMap_->planes()[0].data();
	uint8_t *dst = map.planes()[0].data();
	memcpy(dst, src, ySize);
	memcpy(map.planes().size() > 1 ? map.planes()[1].data() : dst + ySize, src + ySize, ySize / 2);

	ipa_->processOutput(frame);

	request->_d()->metadata().merge(metadata);
	return 0;
}

void IPU7CameraData::rawBufferReady(FrameBuffer *raw)
{
	if (!running_)
		return;

	if (raw->metadata().status != FrameMetadata::FrameSuccess || pendingRequests_.empty()) {
		capture_->queueBuffer(raw);
		return;
	}

	Request *request = pendingRequests_.front();
	pendingRequests_.pop();
	FrameBuffer *out = request->findBuffer(&stream_);

	FrameMetadata &md = out->_d()->metadata();
	md.sequence = raw->metadata().sequence;
	md.timestamp = raw->metadata().timestamp;
	request->_d()->metadata().set(controls::SensorTimestamp, raw->metadata().timestamp);

	int ret = process(raw, request, out);
	md.status = ret ? FrameMetadata::FrameError : FrameMetadata::FrameSuccess;
	if (!ret) {
		Span<FrameMetadata::Plane> planes = md.planes();
		for (unsigned int i = 0; i < planes.size(); i++)
			planes[i].bytesused = out->planes()[i].length;
	}

	capture_->queueBuffer(raw);

	PipelineHandler *pipe = Camera::Private::pipe();
	pipe->completeBuffer(request, out);
	pipe->completeRequest(request);
}

/* -------------------------------------------------------------------------- */

std::unique_ptr<CameraConfiguration>
PipelineHandlerIPU7::generateConfiguration(Camera *camera, Span<const StreamRole> roles)
{
	(void)camera;
	auto config = std::make_unique<IPU7CameraConfiguration>();
	if (roles.empty())
		return config;

	std::map<PixelFormat, std::vector<SizeRange>> formats = {
		{ formats::NV12, { SizeRange(kOutputSize) } },
	};
	StreamConfiguration cfg{ StreamFormats{ formats } };
	cfg.pixelFormat = formats::NV12;
	cfg.size = kOutputSize;
	cfg.bufferCount = kOutputBufferCount;
	config->addConfiguration(cfg);

	if (config->validate() == CameraConfiguration::Invalid)
		return nullptr;
	return config;
}

int PipelineHandlerIPU7::configure(Camera *camera, CameraConfiguration *c)
{
	IPU7CameraData *data = cameraData(camera);
	StreamConfiguration &cfg = c->at(0);

	static const std::array<unsigned int, 1> codes = { MEDIA_BUS_FMT_SRGGB10_1X10 };
	V4L2SubdeviceFormat sensorFormat = data->sensor_->getFormat(codes, kSensorSize);
	int ret = data->sensor_->setFormat(&sensorFormat);
	if (ret)
		return ret;

	V4L2SubdeviceFormat csiFormat = sensorFormat;
	ret = data->csi2_->setFormat(0, &csiFormat);
	if (ret)
		return ret;
	csiFormat = sensorFormat;
	data->csi2_->setFormat(1, &csiFormat);	/* source pad; may be propagated already */

	V4L2DeviceFormat rawFormat;
	rawFormat.fourcc = data->capture_->toV4L2PixelFormat(formats::SRGGB10);
	rawFormat.size = kSensorSize;
	ret = data->capture_->setFormat(&rawFormat);
	if (ret)
		return ret;
	if (rawFormat.size != kSensorSize) {
		LOG(IPU7, Error) << "Capture format rejected: " << rawFormat;
		return -EINVAL;
	}

	/*
	 * \todo Pass real sensor info: CameraSensor::sensorInfo() needs crop selection support the
	 * imx471 driver lacks; the IPA does not use it yet.
	 */
	IPACameraSensorInfo sensorInfo = {};
	sensorInfo.model = data->sensor_->model();
	sensorInfo.outputSize = kSensorSize;
	ipa::ipu7::IPAConfigInfo ipaConfig{ sensorInfo, data->sensor_->controls() };
	ret = data->ipa_->configure(ipaConfig);
	if (ret)
		return ret;

	cfg.setStream(&data->stream_);
	return 0;
}

int PipelineHandlerIPU7::exportFrameBuffers(Camera *camera, Stream *stream,
					    std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	(void)camera;
	unsigned int count = stream->configuration().bufferCount;
	const unsigned int ySize = kOutputSize.width * kOutputSize.height;

	DmaBufAllocator allocator(DmaBufAllocator::DmaBufAllocatorFlag::CmaHeap |
				  DmaBufAllocator::DmaBufAllocatorFlag::SystemHeap |
				  DmaBufAllocator::DmaBufAllocatorFlag::UDmaBuf);
	if (!allocator.isValid()) {
		LOG(IPU7, Error) << "No dma-buf allocator available";
		return -ENODEV;
	}
	return allocator.exportBuffers(count, { ySize, ySize / 2 }, buffers);
}

int PipelineHandlerIPU7::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	IPU7CameraData *data = cameraData(camera);

	int ret = data->psys_.open(kPsysPath);
	if (ret)
		return ret;
	ret = data->psys_.openGraph(data->graphInfo_, data->graphNodes_);
	if (ret)
		goto err_psys;
	ret = data->allocatePsysBuffers();
	if (ret)
		goto err_psys;

	ret = data->capture_->allocateBuffers(kRawBufferCount, &data->rawBuffers_);
	if (ret < 0)
		goto err_psys;
	for (auto &buf : data->rawBuffers_) {
		PSysDevice::Buffer pb;
		ret = data->psys_.import(buf->planes()[0].fd.get(), buf->planes()[0].length, &pb);
		if (ret)
			goto err_raw;
		data->rawPsys_[buf->planes()[0].fd.get()] = pb;
	}

	data->frame_ = 0;	/* the firmware expects frame ids from 0 for every new graph */
	{
		ControlList sensorCtrls;
		ret = data->ipa_->start();
		if (ret)
			goto err_raw;
		data->ipa_->sensorControls(0, &sensorCtrls);
		data->setSensorControls(sensorCtrls);
	}

	ret = data->capture_->streamOn();
	if (ret)
		goto err_raw;
	data->running_ = true;
	for (auto &buf : data->rawBuffers_)
		data->capture_->queueBuffer(buf.get());
	return 0;

err_raw:
	data->rawPsys_.clear();
	data->capture_->releaseBuffers();
	data->rawBuffers_.clear();
err_psys:
	data->psys_.close();
	return ret;
}

void PipelineHandlerIPU7::stopDevice(Camera *camera)
{
	IPU7CameraData *data = cameraData(camera);

	data->running_ = false;
	data->capture_->streamOff();
	data->ipa_->stop();

	while (!data->pendingRequests_.empty()) {
		Request *request = data->pendingRequests_.front();
		data->pendingRequests_.pop();
		cancelRequest(request);
	}

	data->rawPsys_.clear();
	data->capture_->releaseBuffers();
	data->rawBuffers_.clear();
	data->psys_.close();
}

int PipelineHandlerIPU7::queueRequestDevice(Camera *camera, Request *request)
{
	IPU7CameraData *data = cameraData(camera);
	if (!request->findBuffer(&data->stream_))
		return -ENOENT;
	data->pendingRequests_.push(request);
	return 0;
}

bool PipelineHandlerIPU7::match(DeviceEnumerator *enumerator)
{
	/* Only claim the camera when the PSYS (hardware ISP) driver is present and usable. */
	if (access(kPsysPath, R_OK | W_OK)) {
		LOG(IPU7, Debug) << kPsysPath << " not accessible, leaving the camera to other handlers";
		return false;
	}

	DeviceMatch dm("intel-ipu7");
	dm.add(kCsi2Name);
	dm.add(kCaptureName);

	std::shared_ptr<MediaDevice> media = acquireMediaDevice(enumerator, dm);
	if (!media)
		return false;

	if (media->disableLinks())
		return false;
	MediaLink *link = media->link(kCsi2Name, 1, kCaptureName, 0);
	if (!link || link->setEnabled(true))
		return false;

	std::unique_ptr<IPU7CameraData> data = std::make_unique<IPU7CameraData>(this);
	if (data->init(media.get()))
		return false;

	std::set<Stream *> streams = { &data->stream_ };
	const std::string id = data->sensor_->id();
	std::shared_ptr<Camera> camera = Camera::create(std::move(data), id, streams);
	registerCamera(std::move(camera));
	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerIPU7, "ipu7")

} /* namespace libcamera */
