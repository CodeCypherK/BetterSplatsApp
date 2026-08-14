// Per-image quality flagging: which frames get named as hurting the model.
//
// The thresholds are relative to the session's own distribution, which is the
// whole point — Laplacian variance is not comparable between scenes, so an
// absolute cutoff flags every frame in a plain white room and none in a busy
// one. These tests pin that behaviour, and pin the cases where flagging would
// be an outright lie rather than merely a bad guess.

#include <gtest/gtest.h>

#include <string>

#include "final/image_report.h"

namespace bs {
namespace {

ImageQuality Frame(uint32_t id, double lap_var, double overexp = 0.0,
                   uint32_t observations = 400, bool registered = true) {
  ImageQuality q;
  q.frame_id = id;
  q.name = std::to_string(id) + ".jpg";
  q.registered = registered;
  q.observations = observations;
  q.lap_var = lap_var;
  q.overexp_frac = overexp;
  return q;
}

TEST(ImageReport, EmptyIsSafe) {
  const ImageReport report = FlagImages({});
  EXPECT_TRUE(report.images.empty());
  EXPECT_EQ(report.blurry, 0u);
  // And it still produces parseable JSON members rather than a truncated file.
  const std::string json = ImageReportJson(report);
  EXPECT_NE(json.find("\"image_flags\""), std::string::npos);
  EXPECT_NE(json.find("\"images\": ["), std::string::npos);
}

TEST(ImageReport, BlurIsRelativeToTheSessionNotAnAbsoluteNumber) {
  // A dim, low-texture room. Everything reads low; nothing is unusual FOR
  // this room, so nothing should be flagged. An absolute threshold would
  // condemn the whole session.
  std::vector<ImageQuality> plain;
  for (uint32_t i = 1; i <= 10; ++i) plain.push_back(Frame(i, 30.0));
  EXPECT_EQ(FlagImages(plain).blurry, 0u);

  // A richly textured room, same relative spread, one genuinely soft frame.
  std::vector<ImageQuality> busy;
  for (uint32_t i = 1; i <= 10; ++i) busy.push_back(Frame(i, 900.0));
  busy.push_back(Frame(11, 100.0));  // well under half the median
  const ImageReport report = FlagImages(busy);
  EXPECT_EQ(report.blurry, 1u);
  EXPECT_NEAR(report.median_lap_var, 900.0, 1e-9);
}

TEST(ImageReport, MissingSharpnessReadingIsNotBlur) {
  // lap_var == 0 means the capture side recorded nothing — an older session,
  // a frame written before the metric existed. Calling that "blurry" invents
  // a fact about the image.
  std::vector<ImageQuality> images;
  for (uint32_t i = 1; i <= 5; ++i) images.push_back(Frame(i, 500.0));
  images.push_back(Frame(6, 0.0));
  const ImageReport report = FlagImages(images);
  EXPECT_EQ(report.blurry, 0u);
  // ...and the absent reading does not drag the baseline down for the rest.
  EXPECT_NEAR(report.median_lap_var, 500.0, 1e-9);
}

TEST(ImageReport, OverexposureIsAbsolute) {
  // Clipped highlights are destroyed information regardless of what else is
  // in the scene, so this one threshold does not scale with the session.
  std::vector<ImageQuality> images;
  for (uint32_t i = 1; i <= 5; ++i) images.push_back(Frame(i, 500.0, 0.30));
  const ImageReport report = FlagImages(images);
  EXPECT_EQ(report.overexposed, 5u);
}

TEST(ImageReport, UnregisteredFrameIsNotAlsoWeaklyObserved) {
  // A frame with no pose has no observations by definition. Reporting both
  // says the same thing twice and buries the one that matters.
  std::vector<ImageQuality> images;
  for (uint32_t i = 1; i <= 5; ++i) images.push_back(Frame(i, 500.0));
  images.push_back(Frame(6, 500.0, 0.0, /*observations=*/0,
                         /*registered=*/false));
  const ImageReport report = FlagImages(images);
  EXPECT_EQ(report.unregistered, 1u);
  EXPECT_EQ(report.weakly_observed, 0u);
}

TEST(ImageReport, WeaklyObservedUsesRegisteredFramesForTheBaseline) {
  std::vector<ImageQuality> images;
  for (uint32_t i = 1; i <= 8; ++i) images.push_back(Frame(i, 500.0, 0.0, 400));
  images.push_back(Frame(9, 500.0, 0.0, 20));  // 5% of the median
  const ImageReport report = FlagImages(images);
  EXPECT_EQ(report.weakly_observed, 1u);
  EXPECT_NEAR(report.median_observations, 400.0, 1e-9);
}

TEST(ImageReport, SharpFramesAreNeverFlagged) {
  // The failure mode that would make this feature useless: crying wolf on a
  // clean session, so the list gets ignored on the one that matters.
  std::vector<ImageQuality> images;
  for (uint32_t i = 1; i <= 40; ++i) {
    images.push_back(Frame(i, 480.0 + (i % 7) * 10.0, 0.001, 380 + i));
  }
  const ImageReport report = FlagImages(images);
  EXPECT_EQ(report.blurry, 0u);
  EXPECT_EQ(report.overexposed, 0u);
  EXPECT_EQ(report.weakly_observed, 0u);
  EXPECT_EQ(report.unregistered, 0u);
}

TEST(ImageReport, JsonListsFlaggedImagesFirstAndTruncatesHonestly) {
  std::vector<ImageQuality> images;
  for (uint32_t i = 1; i <= 20; ++i) images.push_back(Frame(i, 500.0));
  images.push_back(Frame(99, 500.0, 0.40));  // the one worth seeing
  const ImageReport report = FlagImages(images);

  const std::string json = ImageReportJson(report, /*max_listed=*/3);
  // The flagged frame survives truncation because flagged sorts first.
  EXPECT_NE(json.find("\"frame_id\": 99"), std::string::npos) << json;
  EXPECT_NE(json.find("\"overexposed\""), std::string::npos);
  // The count reflects every image; images_listed says what was printed.
  EXPECT_NE(json.find("\"images_listed\": 3"), std::string::npos) << json;
  // Exactly three rows.
  size_t rows = 0, at = 0;
  while ((at = json.find("\"frame_id\":", at)) != std::string::npos) {
    ++rows;
    at += 1;
  }
  EXPECT_EQ(rows, 3u);
}

}  // namespace
}  // namespace bs
