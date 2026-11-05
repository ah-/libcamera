/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Intel IPU7 ISP parameter generation and 3A (first cut, embedded in the pipeline handler)
 *
 * Generates the LBFF (node 0) parameter payload and lens shading table from gain-indexed
 * register templates plus our own AE / AWB / tone mapping, driven by the hardware statistics.
 * See README.md of the ipu7-re project for the register formats.
 */

#pragma once

#include <array>
#include <stdint.h>
#include <string>
#include <vector>

namespace libcamera::ipa::ipu7 {

class Ipu7Isp
{
public:
	static constexpr size_t kLbffParamsSize = 28288;
	static constexpr size_t kLscSize = 17920;

	/* Tunables, from the IPA tuning file (imx471.yaml); LIBCAMERA_IPU7_* env vars override */
	struct Tuning {
		double aeTarget = 4700;		/* centre-weighted linear RGBS mean (of 65535) */
		double gainMax = 32;		/* total sensor gain cap */
		double minFps = 15;		/* exposure may stretch frames down to this rate */
		double shadowLiftMax = 2.3;	/* cap on tone-curve shadow lift */
		int denoiseOffset = 2;		/* BNLM/XNR template, stops above the gain template */
		double lscLowLightStrength = 0.86;	/* LSC (gain-1) scale at ~60x gain */
	};

	struct SensorSettings {
		int32_t exposure;	/* lines */
		int32_t analogueGain;	/* imx471 code */
		int32_t digitalGain;	/* 256 = 1x */
		int32_t vblank;
	};

	/* Per-template blobs (terminal contents recorded from Intel's stack) */
	struct Bundle {
		std::vector<uint8_t> n0t0, n0t1, n0t2, n0t8, n1t0, n1t1, n1t2, n1t3;
	};

	int init(const std::string &dataDir, const Tuning &tuning);

	SensorSettings sensorSettings() const { return sensor_; }
	double totalGain() const { return curGain_; }
	double exposureUs() const;

	/* Fill all parameter buffers for the next frame. */
	void prepare(uint8_t *lbff, uint8_t *lsc, uint8_t *n0t1, uint8_t *n0t2,
		     uint8_t *n1t0, uint8_t *n1t1, uint8_t *n1t2, uint8_t *n1t3);

	/* Consume LBFF statistics; returns true if the sensor settings changed. */
	bool processStats(const uint16_t *rgbs, const uint8_t *sat, const uint32_t *hist);

	/* Consume the NV12 output (tone curve / white point adaptation). */
	void processOutput(const uint8_t *nv12, unsigned int width, unsigned int height);

	double colourGainR() const { return wbR_; }
	double colourGainB() const { return wbB_; }

private:
	void applyTotal();
	int pickBundle(int cur) const;
	void buildLbff(uint8_t *dst, const uint8_t *tmpl);
	void updateLsc(uint16_t *dst, const uint16_t *tmpl);

	std::array<Bundle, 6> bundles_;
	int bundle_ = -1;

	/* AE */
	double total_ = 1000;	/* exposure lines x total gain */
	double curGain_ = 1;
	int vblank_ = 1203;
	int expMax_ = 2273;
	unsigned int aeFrame_ = 0;
	std::vector<double> aeWeights_;
	SensorSettings sensor_ = { 1000, 0, 256, 1203 };

	/* AWB */
	double wbR_ = 1.2, wbB_ = 2.0;
	unsigned int awbFrames_ = 0;

	/* denoise: BNLM + XNR sections taken from the template nrOffset_ stops up (+) or down (-) */
	int nrOffset_ = 2;
	Tuning tuning_;
	int vblankMax_ = 3500;
	void applyNr(uint8_t *dst) const;

	/* tone */
	double tmMax_ = 2.3;
	double tmStrength_ = 1.0;
	double gk_ = 1.0;

	unsigned int frame_ = 0;
};

} /* namespace libcamera::ipa::ipu7 */
