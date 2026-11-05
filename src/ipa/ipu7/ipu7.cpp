/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * IPA module for the Intel IPU7 hardware ISP
 *
 * Generates the ISP parameters (LBFF register images, lens shading table, program/system
 * terminals from recorded templates) and runs AE / AWB / tone mapping from the hardware
 * statistics. See the ipu7-re project README for the register and statistics formats.
 */

#include <map>
#include <memory>
#include <string>

#include <linux/v4l2-controls.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/framebuffer.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/ipu7_ipa_interface.h>

#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "ipu7_isp.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAIPU7)

namespace ipa::ipu7 {

class IPAIPU7 : public IPAIPU7Interface
{
public:
	int init(const IPASettings &settings) override;
	int configure(const IPAConfigInfo &configInfo) override;
	int start() override;
	void sensorControls(uint32_t frame, ControlList *sensorControls) override;
	void stop() override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void computeParams(uint32_t frame) override;
	void processStats(uint32_t frame, ControlList *sensorControls, ControlList *metadata) override;
	void processOutput(uint32_t frame, const std::vector<uint32_t> &yHistogram) override;

private:
	uint8_t *buffer(BufferId id);
	ControlList sensorControlList() const;

	Ipu7Isp isp_;
	ControlInfoMap sensorControls_;
	std::map<unsigned int, MappedFrameBuffer> mapped_;
	std::map<unsigned int, std::unique_ptr<FrameBuffer>> buffers_;
};

int IPAIPU7::init(const IPASettings &settings)
{
	Ipu7Isp::Tuning tuning;
	std::string templates;

	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(IPAIPU7, Error) << "Failed to open tuning file " << settings.configurationFile;
		return file.error();
	}
	std::unique_ptr<ValueNode> data = YamlParser::parse(file);
	if (!data)
		return -EINVAL;
	if ((*data)["version"].get<uint32_t>(0) != 1) {
		LOG(IPAIPU7, Error) << "Unsupported tuning file version";
		return -EINVAL;
	}

	const ValueNode &ae = (*data)["ae"];
	tuning.aeTarget = ae["target"].get<double>(tuning.aeTarget);
	tuning.gainMax = ae["gain_max"].get<double>(tuning.gainMax);
	tuning.minFps = ae["min_fps"].get<double>(tuning.minFps);
	tuning.shadowLiftMax = (*data)["tone"]["shadow_lift_max"].get<double>(tuning.shadowLiftMax);
	tuning.denoiseOffset = (*data)["denoise"]["template_offset"].get<int32_t>(tuning.denoiseOffset);
	tuning.lscLowLightStrength = (*data)["lsc"]["low_light_strength"].get<double>(tuning.lscLowLightStrength);
	templates = (*data)["templates"].get<std::string>("");

	/* Template directory: relative to the tuning file, LIBCAMERA_IPU7_DATA overrides */
	std::string dir;
	if (const char *env = utils::secure_getenv("LIBCAMERA_IPU7_DATA"))
		dir = env;
	else if (!templates.empty() && templates[0] == '/')
		dir = templates;
	else
		dir = utils::dirname(settings.configurationFile) + "/" + templates;

	LOG(IPAIPU7, Info) << "Tuning " << settings.configurationFile << ", templates " << dir;
	return isp_.init(dir, tuning);
}

int IPAIPU7::configure(const IPAConfigInfo &configInfo)
{
	sensorControls_ = configInfo.sensorControls;
	return 0;
}

ControlList IPAIPU7::sensorControlList() const
{
	const Ipu7Isp::SensorSettings s = isp_.sensorSettings();
	ControlList ctrls(sensorControls_);
	ctrls.set(V4L2_CID_VBLANK, s.vblank);
	ctrls.set(V4L2_CID_EXPOSURE, s.exposure);
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, s.analogueGain);
	ctrls.set(V4L2_CID_DIGITAL_GAIN, s.digitalGain);
	return ctrls;
}

int IPAIPU7::start()
{
	return 0;
}

void IPAIPU7::sensorControls([[maybe_unused]] uint32_t frame, ControlList *sensorControls)
{
	*sensorControls = sensorControlList();
}

void IPAIPU7::stop()
{
}

void IPAIPU7::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &b : buffers) {
		auto fb = std::make_unique<FrameBuffer>(b.planes);
		MappedFrameBuffer map(fb.get(), MappedFrameBuffer::MapFlag::ReadWrite);
		if (!map.isValid()) {
			LOG(IPAIPU7, Error) << "Failed to map buffer " << b.id;
			continue;
		}
		mapped_.emplace(b.id, std::move(map));
		buffers_.emplace(b.id, std::move(fb));
	}
}

void IPAIPU7::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		mapped_.erase(id);
		buffers_.erase(id);
	}
}

uint8_t *IPAIPU7::buffer(BufferId id)
{
	auto it = mapped_.find(static_cast<unsigned int>(id));
	if (it == mapped_.end())
		return nullptr;
	return it->second.planes()[0].data();
}

void IPAIPU7::computeParams([[maybe_unused]] uint32_t frame)
{
	uint8_t *p[] = { buffer(BufferId::LbffParams), buffer(BufferId::LscTable),
			 buffer(BufferId::LbffProgram), buffer(BufferId::LbffSystem),
			 buffer(BufferId::BbpsParams), buffer(BufferId::BbpsProgram),
			 buffer(BufferId::BbpsSystem), buffer(BufferId::BbpsFrame) };
	for (uint8_t *x : p)
		if (!x)
			return;
	isp_.prepare(p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
}

void IPAIPU7::processStats([[maybe_unused]] uint32_t frame, ControlList *sensorControls,
			   ControlList *metadata)
{
	const uint8_t *ae = buffer(BufferId::StatsAe);
	const uint8_t *rgbs = buffer(BufferId::StatsRgbs);
	const uint8_t *sat = buffer(BufferId::StatsSat);
	if (!ae || !rgbs || !sat)
		return;

	if (isp_.processStats(reinterpret_cast<const uint16_t *>(rgbs), sat,
			      reinterpret_cast<const uint32_t *>(ae)))
		*sensorControls = sensorControlList();

	metadata->set(controls::ExposureTime, static_cast<int32_t>(isp_.exposureUs()));
	metadata->set(controls::AnalogueGain, static_cast<float>(isp_.totalGain()));
	metadata->set(controls::ColourGains, { static_cast<float>(isp_.colourGainR()),
					       static_cast<float>(isp_.colourGainB()) });
}

void IPAIPU7::processOutput([[maybe_unused]] uint32_t frame, const std::vector<uint32_t> &yHistogram)
{
	if (yHistogram.size() == 256)
		isp_.processOutput(yHistogram.data());
}

} /* namespace ipa::ipu7 */

extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"ipu7",
	"ipu7",
};

IPAInterface *ipaCreate()
{
	return new ipa::ipu7::IPAIPU7();
}
}

} /* namespace libcamera */
