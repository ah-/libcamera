/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Intel IPU7 ISP parameter generation and 3A (first cut)
 *
 * Port of live2.c from the ipu7-re project. Register formats (LBFF payload offsets):
 *   WB_1_1      @0x2f2c  u32 [3]=B [4]=R gains, Q3.12
 *   CCM_3A_2_0  @0x2990  u16 words 2-5 (+copy 6-9) statistics WB gains Gr,R,B,Gb, Q8
 *   GammaTM_V4  @0x4b9c  15-bit samples, 2 per u32; 27..1562 gamma LUT, 1585..3632 tone-gain LUT (Q12)
 *   LSC table (term 8)   35 x 64 x {Gr,R,B,Gb} u16, gain = 1 + v/2048
 * Statistics: RGBS grid 60x68 blocks x {Gr,R,B,Gb} u16 (post-LSC, pre-WB); saturation u8 per
 * block; 4 x 256-bin u32 histograms, h3 = luma.
 */

#include "ipu7_isp.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdlib.h>
#include <string.h>

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(IPAIPU7)

namespace ipa::ipu7 {

namespace {

constexpr int kOffWb = 0x2f2c;
constexpr int kOffCcm3a = 0x2990;
constexpr int kOffGtm = 0x4b9c;
constexpr int kTmFirst = 1585;
constexpr int kTmN = 2048;
constexpr int kGridW = 60;
constexpr int kGridH = 68;

constexpr int kVblankMin = 1203;
constexpr double kLineTimeUs = 2328.0 / 160.0;	/* (width 1928 + hblank 400) / pixel rate 160 MHz */

constexpr double kBundleGain[6] = { 1, 2, 4, 8, 16, 32 };
const char *const kBundleDir[6] = { "g0", "g6", "g12", "g18", "g24", "g30" };

/* BNLM_3_3 and XNR_5_2 payload sections (offset, size) from the lbff_0 descriptors */
constexpr int kNrSections[][2] = {
	{ 0x2f5c, 0x2d4 }, { 0x3230, 0x100 },
	{ 0x36c4, 0xc }, { 0x36d0, 0x3c }, { 0x370c, 0x18 }, { 0x3724, 0x1a8 }, { 0x38cc, 0x180 },
	{ 0x3a4c, 0x38 }, { 0x3a84, 0x19c }, { 0x3c20, 0x180 }, { 0x3da0, 0x4 }, { 0x3da4, 0x1c },
	{ 0x3dc0, 0x180 }, { 0x3f40, 0x180 }, { 0x40c0, 0x1c }, { 0x40dc, 0x1c }, { 0x40f8, 0x48 },
	{ 0x4140, 0x148 }, { 0x4288, 0x180 },
};

uint32_t rd32(const uint8_t *p, int off) { uint32_t v; memcpy(&v, p + off, 4); return v; }
void wr32(uint8_t *p, int off, uint32_t v) { memcpy(p + off, &v, 4); }

int gtmGet(const uint8_t *p, int i)
{
	uint32_t w = rd32(p, kOffGtm + (i / 2) * 4);
	return (i & 1) ? (w >> 15) & 0x7fff : w & 0x7fff;
}

void gtmSet(uint8_t *p, int i, int v)
{
	v = std::clamp(v, 0, 0x7fff);
	uint32_t w = rd32(p, kOffGtm + (i / 2) * 4);
	if (i & 1)
		w = (w & ~(0x7fffu << 15)) | (uint32_t(v) << 15);
	else
		w = (w & ~0x7fffu) | uint32_t(v);
	wr32(p, kOffGtm + (i / 2) * 4, w);
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

int Ipu7Isp::init(const std::string &dataDir, const Tuning &tuning)
{
	tuning_ = tuning;
	static const char *const names[] = { "n0t0", "n0t1", "n0t2", "n0t8", "n1t0", "n1t1", "n1t2", "n1t3" };
	for (unsigned int b = 0; b < bundles_.size(); b++) {
		Bundle &x = bundles_[b];
		std::vector<uint8_t> *dst[] = { &x.n0t0, &x.n0t1, &x.n0t2, &x.n0t8, &x.n1t0, &x.n1t1, &x.n1t2, &x.n1t3 };
		for (unsigned int k = 0; k < 8; k++) {
			std::string path = dataDir + "/bundles/" + kBundleDir[b] + "/" + names[k] + ".bin";
			if (!readFile(path, dst[k])) {
				LOG(IPAIPU7, Error) << "Missing template " << path;
				return -ENOENT;
			}
		}
		if (x.n0t0.size() != kLbffParamsSize || x.n0t8.size() != kLscSize) {
			LOG(IPAIPU7, Error) << "Template " << kBundleDir[b] << " has unexpected size";
			return -EINVAL;
		}
	}

	/* seed white balance from the 4x template */
	const uint8_t *t = bundles_[2].n0t0.data();
	wbB_ = rd32(t, kOffWb + 12) / 4096.0;
	wbR_ = rd32(t, kOffWb + 16) / 4096.0;

	aeWeights_.resize(kGridW * kGridH);
	for (int y = 0; y < kGridH; y++)
		for (int x = 0; x < kGridW; x++) {
			double dx = (x - 29.5) / 20, dy = (y - 33.5) / 22;
			aeWeights_[y * kGridW + x] = 1 + 1.5 * std::exp(-(dx * dx + dy * dy));
		}

	/* live-tuning overrides */
	if (const char *e = getenv("LIBCAMERA_IPU7_NR"))
		tuning_.denoiseOffset = atoi(e);
	if (const char *e = getenv("LIBCAMERA_IPU7_MIN_FPS"))
		tuning_.minFps = atof(e);
	if (const char *e = getenv("LIBCAMERA_IPU7_TM_MAX"))
		tuning_.shadowLiftMax = atof(e);

	nrOffset_ = std::clamp(tuning_.denoiseOffset, -5, 5);
	tmMax_ = std::clamp(tuning_.shadowLiftMax, 0.5, 3.0);
	tuning_.gainMax = std::clamp(tuning_.gainMax, 1.0, 64.0);
	/* frame length (lines) = 1088 + VBLANK, line time 14.55 us */
	vblankMax_ = std::max(kVblankMin, static_cast<int>(1e6 / (std::clamp(tuning_.minFps, 5.0, 30.0) * kLineTimeUs)) - 1088);

	LOG(IPAIPU7, Info) << "AE target " << tuning_.aeTarget << ", gain cap " << tuning_.gainMax
			   << "x, min " << tuning_.minFps << " fps, denoise offset " << nrOffset_
			   << ", shadow lift cap " << tmMax_;

	applyTotal();
	return 0;
}

void Ipu7Isp::applyNr(uint8_t *dst) const
{
	if (!nrOffset_)
		return;
	int b = std::clamp(bundle_ + nrOffset_, 0, 5);
	if (b == bundle_)
		return;
	const uint8_t *src = bundles_[b].n0t0.data();
	for (const auto &s : kNrSections)
		memcpy(dst + s[0], src + s[0], s[1]);
}

double Ipu7Isp::exposureUs() const
{
	return sensor_.exposure * kLineTimeUs;
}

void Ipu7Isp::applyTotal()
{
	/* exposure first: stretch the frame (lower fps) before raising gain beyond 4x */
	int wantVb = kVblankMin;
	if (total_ > (kVblankMin + 1070) * 4.0)
		wantVb = static_cast<int>(std::min<double>(vblankMax_, total_ / 4.0 - 1070));
	if (std::abs(wantVb - vblank_) > 50)
		vblank_ = wantVb;
	expMax_ = vblank_ + 1070;

	total_ = std::min(total_, expMax_ * tuning_.gainMax);
	int exp = total_ > expMax_ ? expMax_ : (total_ < 4 ? 4 : static_cast<int>(total_));
	double rest = total_ / exp;
	double ag = std::clamp(rest, 1.0, 16.0);
	int agc = std::min(960, static_cast<int>(std::lround(1024 - 1024 / ag)));
	double agr = 1024.0 / (1024 - agc);
	double dg = std::clamp(rest / agr, 1.0, tuning_.gainMax / 16.0);
	int dgc = static_cast<int>(std::lround(dg * 256));
	curGain_ = agr * dgc / 256.0;

	sensor_ = { exp, agc, dgc, vblank_ };
}

int Ipu7Isp::pickBundle(int cur) const
{
	double lg = std::log2(curGain_);
	if (cur >= 0 && std::fabs(lg - std::log2(kBundleGain[cur])) < 0.7)
		return cur;
	int best = 0;
	for (int i = 1; i < 6; i++)
		if (std::fabs(lg - std::log2(kBundleGain[i])) < std::fabs(lg - std::log2(kBundleGain[best])))
			best = i;
	return best;
}

void Ipu7Isp::buildLbff(uint8_t *dst, const uint8_t *tmpl)
{
	memcpy(dst, tmpl, kLbffParamsSize);

	/* white balance (image path) */
	wr32(dst, kOffWb + 12, static_cast<uint32_t>(std::lround(wbB_ * 4096)));
	wr32(dst, kOffWb + 16, static_cast<uint32_t>(std::lround(wbR_ * 4096)));

	/* statistics-path WB gains follow ours so the AE histograms see white-balanced data */
	uint16_t gq[4] = { 256, static_cast<uint16_t>(std::min<long>(std::lround(wbR_ * 256), 1023)),
			   static_cast<uint16_t>(std::min<long>(std::lround(wbB_ * 256), 1023)), 256 };
	memcpy(dst + kOffCcm3a + 4, gq, 8);
	memcpy(dst + kOffCcm3a + 12, gq, 8);

	/* tone gains, scaled in the log domain (templates contain 0x gains for the noise floor) */
	for (int i = 1; i < kTmN; i++) {
		int g = gtmGet(tmpl, kTmFirst + i);
		gtmSet(dst, kTmFirst + i, g > 0 ? static_cast<int>(std::lround(4096 * std::pow(g / 4096.0, tmStrength_))) : 0);
	}

	/* white-point stretch on the gamma LUT */
	if (gk_ != 1.0)
		for (int i = 27; i < 27 + 1536; i++)
			gtmSet(dst, i, static_cast<int>(std::lround(gtmGet(tmpl, i) * gk_)));
}

void Ipu7Isp::updateLsc(uint16_t *dst, const uint16_t *tmpl)
{
	/* full strength up to 16x gain, easing to 0.86 at ~60x (Intel's low-light behaviour) */
	double s = curGain_ <= 16 ? 1.0 : std::max(tuning_.lscLowLightStrength, 1.0 - (1.0 - tuning_.lscLowLightStrength) * std::log2(curGain_ / 16) / 2);
	for (size_t i = 0; i < kLscSize / 2; i++)
		dst[i] = static_cast<uint16_t>(std::min<double>(std::lround(tmpl[i] * s), 65535));
}

void Ipu7Isp::prepare(uint8_t *lbff, uint8_t *lsc, uint8_t *n0t1, uint8_t *n0t2,
		      uint8_t *n1t0, uint8_t *n1t1, uint8_t *n1t2, uint8_t *n1t3)
{
	int nb = pickBundle(bundle_);
	if (nb != bundle_) {
		LOG(IPAIPU7, Debug) << "Template " << kBundleDir[nb] << " for gain " << curGain_;
		bundle_ = nb;
	}
	const Bundle &b = bundles_[bundle_];

	buildLbff(lbff, b.n0t0.data());
	applyNr(lbff);
	updateLsc(reinterpret_cast<uint16_t *>(lsc), reinterpret_cast<const uint16_t *>(b.n0t8.data()));
	memcpy(n0t1, b.n0t1.data(), b.n0t1.size());
	memcpy(n0t2, b.n0t2.data(), b.n0t2.size());
	memcpy(n1t0, b.n1t0.data(), b.n1t0.size());
	memcpy(n1t1, b.n1t1.data(), b.n1t1.size());
	memcpy(n1t2, b.n1t2.data(), b.n1t2.size());
	memcpy(n1t3, b.n1t3.data(), b.n1t3.size());
}

bool Ipu7Isp::processStats(const uint16_t *rgbs, const uint8_t *sat, const uint32_t *hist)
{
	frame_++;

	/* ---- AWB: two-pass grey world on the RGBS grid (pre-WB, post-LSC) */
	{
		double lr = std::log(wbR_), lb = std::log(wbB_);
		long used = 0;
		bool ok = true;
		for (int pass = 0; pass < 2 && ok; pass++) {
			double sr = 0, sg = 0, sb = 0;
			used = 0;
			for (int i = 0; i < kGridW * kGridH; i++) {
				double gr = rgbs[i * 4], r = rgbs[i * 4 + 1], b = rgbs[i * 4 + 2], gb = rgbs[i * 4 + 3];
				double g = 0.5 * (gr + gb);
				if (g < 1536 || g > 60000 || r > 60000 || b > 60000 || sat[i] || r < 8 || b < 8)
					continue;
				if (pass && (std::fabs(std::log(r / g) + lr) > 0.35 || std::fabs(std::log(b / g) + lb) > 0.35))
					continue;
				sr += r; sg += g; sb += b; used++;
			}
			if (used < 120) {
				ok = false;	/* too dark: hold the last estimate */
				break;
			}
			lr = std::log(sg / sr);
			lb = std::log(sg / sb);
		}
		if (ok) {
			/* keep some warmth under warm light (Intel's rendering does this) */
			double k = std::clamp(lb - lr - std::log(1.5), 0.0, 1.5);
			double tr = lr + 0.04 * k, tb = lb - 0.06 * k;
			double a = awbFrames_++ < 10 ? 0.5 : 0.12;
			wbR_ = std::exp(std::log(wbR_) + a * (tr - std::log(wbR_)));
			wbB_ = std::exp(std::log(wbB_) + a * (tb - std::log(wbB_)));
		}
	}

	/* ---- AE: centre-weighted RGBS luminance + luma-histogram highlight protection */
	const uint32_t *h3 = hist + 3 * 256;
	double n = 0, clip = 0;
	for (int k = 0; k < 256; k++) {
		n += h3[k];
		if (k >= 250)
			clip += h3[k];
	}
	double acc = 0, p995 = 255;
	for (int k = 0; k < 256; k++) {
		acc += h3[k];
		if (acc >= 0.995 * n) {
			p995 = k;
			break;
		}
	}
	double clipFrac = n > 0 ? clip / n : 0;

	double sw = 0, sl = 0;
	int nsat = 0;
	int lhist[257] = {};
	for (int i = 0; i < kGridW * kGridH; i++) {
		double r = rgbs[i * 4 + 1] * wbR_, g = 0.5 * (rgbs[i * 4] + rgbs[i * 4 + 3]), b = rgbs[i * 4 + 2] * wbB_;
		double L = std::min(0.2126 * r + 0.7152 * g + 0.0722 * b, 65535.0);
		sl += L * aeWeights_[i];
		sw += aeWeights_[i];
		nsat += sat[i] > 0;
		lhist[static_cast<int>(L / 256.01)]++;
	}
	double cwMean = sl / sw;
	int cacc = 0;
	double p99 = 65535;
	for (int k = 0; k < 256; k++) {
		cacc += lhist[k];
		if (cacc >= 0.99 * kGridW * kGridH) {
			p99 = (k + 1) * 256.0;
			break;
		}
	}

	if (++aeFrame_ % 3)
		return false;	/* the sensor applies new settings ~2 frames later */

	double f = tuning_.aeTarget / std::max(cwMean, 1.0);
	if (p99 > 45000 || nsat > 20)
		f = std::min(f, 45000 / std::max(p99, 1.0) * (nsat > 20 ? 0.9 : 1));
	if (clipFrac > 0.005)
		f = std::min(f, 0.85);
	else if (p995 > 235)
		f = std::min(f, 230.0 / p995);
	f = std::exp(0.6 * std::log(std::clamp(f, 0.25, 4.0)));

	if (std::fabs(std::log(f)) <= 0.03)
		return false;

	SensorSettings old = sensor_;
	total_ = std::max(total_ * f, 4.0);
	applyTotal();
	LOG(IPAIPU7, Debug) << "AE cw " << cwMean << " p99 " << p99 << " -> exp " << sensor_.exposure
			 << " ag " << sensor_.analogueGain << " dg " << sensor_.digitalGain
			 << " vb " << sensor_.vblank << " wb " << wbR_ << "/" << wbB_;
	return memcmp(&old, &sensor_, sizeof(old)) != 0;
}

void Ipu7Isp::processOutput(const uint8_t *nv12, unsigned int width, unsigned int height)
{
	if (frame_ % 2)
		return;

	long n = 0, hi = 0, dark = 0;
	long hist[256] = {};
	for (unsigned int r = 0; r < height; r += 8)
		for (unsigned int c = 0; c < width; c += 8) {
			int v = nv12[r * width + c];
			n++;
			hist[v]++;
			hi += v > 228;
			dark += v < 45;
		}
	double hif = double(hi) / n, darkf = double(dark) / n;

	/* white point: push the 99th percentile towards 228 (limited range white is 235) */
	long acc = 0;
	int p99 = 255;
	for (int k = 0; k < 256; k++) {
		acc += hist[k];
		if (acc >= n * 0.99) {
			p99 = k;
			break;
		}
	}
	double wantGk = std::clamp(gk_ * 228.0 / std::max(p99, 16), 1.0, 1.5);
	gk_ += 0.08 * (wantGk - gk_);

	/* lift shadows harder when much of the frame is dark */
	double want = std::min(0.8 + 1.2 * std::min(darkf / 0.4, 1.0) + (hif > 0.01 ? 0.3 : 0), tmMax_);
	tmStrength_ += 0.1 * (want - tmStrength_);
}

} /* namespace ipa::ipu7 */

} /* namespace libcamera */
