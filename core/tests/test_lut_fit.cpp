#include <gtest/gtest.h>

#include "calib/lut_fit.h"

namespace bs {
namespace {

const PinholeIntrinsics kIntrinsics{1456.0, 1456.0, 959.5, 719.5, 1920, 1440};

double MaxRadiusPx() {
  const double dx = std::max(kIntrinsics.cx, 1920 - kIntrinsics.cx);
  const double dy = std::max(kIntrinsics.cy, 1440 - kIntrinsics.cy);
  return std::hypot(dx, dy);
}

// Builds Apple-style tables from a known OPENCV model.
//
// Apple's entries are RELATIVE magnification — the fraction by which the
// radius changes — so a table entry is (ratio - 1), and the entry at the
// optical centre is 0. This fixture used to emit the ratio itself, which is
// the same misunderstanding the implementation had, so the two agreed with
// each other and the test could only ever confirm the bug. Real device
// tables start at 0.0; a ratio table would start at 1.0, and that
// difference is what reached a user as 2 of 431 photos placed.
DistortionLut MakeLutFromModel(double k1, double k2, int entries = 42) {
  DistortionLut lut;
  lut.center_x = kIntrinsics.cx;
  lut.center_y = kIntrinsics.cy;
  const double r_max = MaxRadiusPx();
  const double f = kIntrinsics.fx;

  // Inverse table: index by undistorted radius fraction; entry = r_d / r_u.
  for (int i = 0; i < entries; ++i) {
    const double rf = static_cast<double>(i) / (entries - 1);
    const double ru = rf * r_max / f;
    const double scale = 1.0 + k1 * ru * ru + k2 * ru * ru * ru * ru;
    lut.inverse.push_back(static_cast<float>(scale - 1.0));
  }
  // Forward table: index by distorted radius fraction; entry = r_u / r_d.
  // Invert numerically.
  for (int i = 0; i < entries; ++i) {
    const double rf = static_cast<double>(i) / (entries - 1);
    const double rd = rf * r_max / f;
    double ru = rd;
    for (int it = 0; it < 25; ++it) {
      const double scale = 1.0 + k1 * ru * ru + k2 * ru * ru * ru * ru;
      ru = rd / scale;
    }
    lut.magnification.push_back(
        static_cast<float>(rd > 1e-12 ? ru / rd - 1.0 : 0.0));
  }
  return lut;
}

TEST(LutFit, RecoversKnownModelFromInverseTable) {
  const double k1 = 0.06, k2 = -0.11;
  DistortionLut lut = MakeLutFromModel(k1, k2);
  lut.magnification.clear();  // force the inverse-table path

  const auto fit = FitOpencvModelFromLut(lut, kIntrinsics);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->k1, k1, 2e-3);
  EXPECT_NEAR(fit->k2, k2, 5e-3);
  EXPECT_LT(fit->max_residual_px, 0.25);
}

TEST(LutFit, RecoversKnownModelFromForwardTable) {
  const double k1 = 0.045, k2 = -0.08;
  DistortionLut lut = MakeLutFromModel(k1, k2);
  lut.inverse.clear();  // force the forward-table path

  const auto fit = FitOpencvModelFromLut(lut, kIntrinsics);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->k1, k1, 4e-3);
  EXPECT_NEAR(fit->k2, k2, 1e-2);
  EXPECT_LT(fit->max_residual_px, 0.35);
}

TEST(LutFit, EmptyLutIsExactPinhole) {
  const auto fit = FitOpencvModelFromLut(DistortionLut{}, kIntrinsics);
  ASSERT_TRUE(fit.has_value());
  EXPECT_DOUBLE_EQ(fit->k1, 0.0);
  EXPECT_DOUBLE_EQ(fit->k2, 0.0);
  EXPECT_DOUBLE_EQ(fit->max_residual_px, 0.0);
}

TEST(LutFit, RejectsDegenerateIntrinsics) {
  const DistortionLut lut = MakeLutFromModel(0.05, -0.1);
  PinholeIntrinsics bad = kIntrinsics;
  bad.fx = 0;
  EXPECT_FALSE(FitOpencvModelFromLut(lut, bad).has_value());
}

TEST(LutFit, UndistortInvertsDistort) {
  const double k1 = 0.06, k2 = -0.11;
  for (const auto& px : {Eigen::Vector2d(100, 100), Eigen::Vector2d(959, 719),
                         Eigen::Vector2d(1800, 1350), Eigen::Vector2d(40, 1400)}) {
    // Treat px as undistorted; distort it, then undistort back.
    const Eigen::Vector2d xy_u((px.x() - kIntrinsics.cx) / kIntrinsics.fx,
                               (px.y() - kIntrinsics.cy) / kIntrinsics.fy);
    const Eigen::Vector2d xy_d = DistortNormalized(xy_u, k1, k2);
    const Eigen::Vector2d px_d(xy_d.x() * kIntrinsics.fx + kIntrinsics.cx,
                               xy_d.y() * kIntrinsics.fy + kIntrinsics.cy);
    const Eigen::Vector2d roundtrip = UndistortPixel(px_d, kIntrinsics, k1, k2);
    EXPECT_LT((roundtrip - px).norm(), 5e-3) << px.transpose();
  }
}


// Regression: a real iPhone lens table, from the capture that exposed the
// bug. Everything above this uses synthetic data, which carries no LUT at
// all and returns early as an exact pinhole — so this arithmetic had never
// once run on real calibration until a device produced a reconstruction
// with 2 of 431 photos placed.
//
// The table was applied as a direct multiplier instead of as a RELATIVE
// magnification (1 + value). Apple's values here peak at 1.5%, so every
// distorted radius collapsed to about a hundredth of its true length, and
// the fit answered with k1 = -5.04, k2 = +5.54 and a 252 px worst-case
// error — mapping a 900 px radius to a NEGATIVE one. With that, every
// undistorted point in the pipeline is wrong, which takes two-view
// geometry, PnP and triangulation with it.
TEST(LutFit, RealIPhoneTableFitsAPlausibleLens) {
  DistortionLut lut;
  lut.center_x = 958.6969895;
  lut.center_y = 720.3926326;
  lut.magnification = {
      0.0f, -0.000103547995f, -0.00041134737f, -0.0009149136f, -0.0016002714f, -0.002448207f,
      -0.0034346282f, -0.0045310394f, -0.005705132f, -0.006921499f, -0.008142465f, -0.009329037f,
      -0.0104419505f, -0.011442812f, -0.012295298f, -0.012966391f, -0.013427606f, -0.013656176f,
      -0.013636129f, -0.013359207f, -0.012825552f, -0.012044104f, -0.011032612f, -0.009817212f,
      -0.0084315f, -0.0069150673f, -0.0053114914f, -0.0036658293f, -0.0020217143f, -0.00041823712f,
      0.0011131488f, 0.0025513773f, 0.0038878168f, 0.0051272316f, 0.0062873093f, 0.0073964787f,
      0.008489895f, 0.009603687f, 0.0107678985f, 0.011998979f, 0.013293348f, 0.0146244075f};
  lut.inverse = {
      0.0f, 0.00010382716f, 0.00041269293f, 0.0009187629f, 0.0016090265f, 0.0024653834f,
      0.0034647943f, 0.004579524f, 0.005777511f, 0.0070228875f, 0.008276691f, 0.009497776f,
      0.010643947f, 0.011673292f, 0.012545706f, 0.013224527f, 0.013678244f, 0.013882142f,
      0.013819792f, 0.01348427f, 0.0128789395f, 0.012017744f, 0.010924868f, 0.009633759f,
      0.008185469f, 0.0066263815f, 0.005005408f, 0.0033707893f, 0.0017667152f, 0.00022997918f,
      -0.0012130402f, -0.002548761f, -0.0037777978f, -0.0049150283f, -0.005987996f, -0.0070333937f,
      -0.008091553f, -0.009199105f, -0.010380451f, -0.011639198f, -0.012951548f, -0.014264471f};
  PinholeIntrinsics K;
  K.fx = 1438.316528;
  K.fy = 1438.316528;
  K.cx = 957.4606934;
  K.cy = 720.9101562;
  K.ref_w = 1920;
  K.ref_h = 1440;

  const auto fit = FitOpencvModelFromLut(lut, K);
  ASSERT_TRUE(fit.has_value());
  EXPECT_FALSE(fit->rejected) << "a real lens must fit better than nothing";

  // Phone lenses live in this range. The broken version produced -5.04.
  EXPECT_LT(std::abs(fit->k1), 0.5) << "k1 = " << fit->k1;
  EXPECT_LT(std::abs(fit->k2), 0.5) << "k2 = " << fit->k2;

  // And the model must beat leaving the image uncorrected by a wide margin.
  // Uncorrected, this lens is ~17 px out at the corner; the broken fit was
  // 252 px out, i.e. far worse than doing nothing.
  EXPECT_LT(fit->max_residual_px, 10.0)
      << "worst-case residual " << fit->max_residual_px << " px";

  // The forward model must stay monotonic and positive across the frame —
  // the broken fit sent r = 900 px to a negative radius.
  double previous = 0;
  for (double r_px = 50; r_px <= 1200; r_px += 50) {
    const Eigen::Vector2d d =
        DistortNormalized({r_px / K.fx, 0.0}, fit->k1, fit->k2);
    const double r_d = d.x() * K.fx;
    EXPECT_GT(r_d, 0.0) << "r_u = " << r_px << " px mapped to " << r_d;
    EXPECT_GT(r_d, previous) << "not monotonic at r_u = " << r_px;
    EXPECT_LT(std::abs(r_d - r_px), 40.0) << "r_u = " << r_px;
    previous = r_d;
  }
}

// A fit that comes out non-physical must be REFUSED, not applied.
// Uncorrected geometry is merely imprecise; wrongly corrected geometry
// silently poisons every stage downstream, which is exactly what happened.
//
// Note this is the gate that would have caught the original bug. An
// "is it better than doing nothing" check would not have: k1 = k2 = 0 is
// inside the model space, so least squares can essentially never do worse
// in the mean, and with the broken input BOTH sides of that comparison were
// nonsense anyway.
TEST(LutFit, ANonPhysicalFitIsRejectedRatherThanApplied) {
  DistortionLut lut;
  lut.center_x = 960;
  lut.center_y = 720;
  // The shape the bug produced: a radius that collapses towards zero.
  for (int i = 0; i < 42; ++i) {
    const double rf = static_cast<double>(i) / 41.0;
    lut.inverse.push_back(static_cast<float>(-0.97 * rf));
  }
  PinholeIntrinsics K;
  K.fx = K.fy = 1438.0;
  K.cx = 960;
  K.cy = 720;
  K.ref_w = 1920;
  K.ref_h = 1440;

  const auto fit = FitOpencvModelFromLut(lut, K);
  ASSERT_TRUE(fit.has_value());
  EXPECT_TRUE(fit->rejected);
  EXPECT_EQ(fit->k1, 0.0);
  EXPECT_EQ(fit->k2, 0.0);
}

// The same judgement, reachable on a model that did not come from a fit.
// calibration.json is an input: it can have been written by an older build
// whose fit was wrong, which is exactly the session most in need of being
// re-solved. Trusting a persisted model more than a freshly computed one
// has the check backwards.
//
// The stakes are asymmetric and measured. On the synthetic lens scene the
// live pass tracks 31.7% of frames with the correct model and 28.3% with no
// correction at all — but with the device's broken k1 = -5.04 it never
// bootstraps: 0 keyframes, 0 points, 0 tracked poses out of 60. Falling back
// to an uncorrected pinhole costs accuracy. Applying a wrong model costs
// everything.
TEST(LutFit, APersistedModelIsJudgedTheSameWayAFittedOneIs) {
  PinholeIntrinsics K;
  K.fx = K.fy = 1438.316528;
  K.cx = 957.4606934;
  K.cy = 720.9101562;
  K.ref_w = 1920;
  K.ref_h = 1440;

  // What the device actually wrote down.
  EXPECT_FALSE(IsPhysicalLensModel(-5.04, 5.54, K));
  // What the fixed fit produces for the same lens.
  EXPECT_TRUE(IsPhysicalLensModel(0.0415, -0.0976, K));
  // No distortion is always a legal claim.
  EXPECT_TRUE(IsPhysicalLensModel(0.0, 0.0, K));
  // Strong but real barrel/pincushion stays allowed — the gate rejects
  // impossible lenses, not merely unusual ones.
  EXPECT_TRUE(IsPhysicalLensModel(-0.15, 0.05, K));
  // A model that folds the frame back on itself does not.
  EXPECT_FALSE(IsPhysicalLensModel(-3.0, 0.0, K));
  // Degenerate intrinsics cannot be checked, so they are not trusted.
  EXPECT_FALSE(IsPhysicalLensModel(0.05, 0.0, PinholeIntrinsics{}));
}

}  // namespace
}  // namespace bs
