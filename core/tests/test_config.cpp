#include <gtest/gtest.h>

#include "common/config.h"

namespace bs {
namespace {

TEST(Config, DefaultsWhenEmpty) {
  bool ok = false;
  EngineConfig c = EngineConfig::FromJson("{}", &ok);
  EXPECT_TRUE(ok);
  EXPECT_EQ(c.live_orb_features, 1200);
  EXPECT_FLOAT_EQ(c.patch_size_m, 0.35f);
  EXPECT_EQ(c.final_ba_rounds, 3);

  c = EngineConfig::FromJson(nullptr, &ok);
  EXPECT_TRUE(ok);
  EXPECT_EQ(c.live_orb_features, 1200);
}

TEST(Config, OverridesApply) {
  bool ok = false;
  EngineConfig c = EngineConfig::FromJson(
      R"({"live_orb_features": 800, "patch_size_m": 0.5, "final_ba_rounds": 2})",
      &ok);
  EXPECT_TRUE(ok);
  EXPECT_EQ(c.live_orb_features, 800);
  EXPECT_FLOAT_EQ(c.patch_size_m, 0.5f);
  EXPECT_EQ(c.final_ba_rounds, 2);
}

TEST(Config, UnknownKeysIgnored) {
  bool ok = false;
  EngineConfig c = EngineConfig::FromJson(R"({"not_a_real_key": 42})", &ok);
  EXPECT_TRUE(ok);
  EXPECT_EQ(c.live_orb_features, 1200);
}

TEST(Config, MalformedFallsBackToDefaults) {
  bool ok = true;
  EngineConfig c = EngineConfig::FromJson("{nope", &ok);
  EXPECT_FALSE(ok);
  EXPECT_EQ(c.live_orb_features, 1200);

  ok = true;
  c = EngineConfig::FromJson("[1,2,3]", &ok);
  EXPECT_FALSE(ok);
}

TEST(Config, WrongTypeKeepsDefault) {
  bool ok = false;
  EngineConfig c =
      EngineConfig::FromJson(R"({"live_orb_features": "many"})", &ok);
  EXPECT_TRUE(ok);  // parse ok; bad value is skipped with a warning
  EXPECT_EQ(c.live_orb_features, 1200);
}

}  // namespace
}  // namespace bs
