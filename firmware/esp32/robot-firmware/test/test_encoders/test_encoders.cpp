#include <gtest/gtest.h>

#include <random>

#include "drivers/axis_map.hpp"
#include "drivers/counter_extender.hpp"
#include "hal_fakes/fakes.hpp"

namespace {

using robot::drivers::AxisMap;
using robot::drivers::CounterExtender;
using robot::drivers::Vector3;
using robot::hal_fakes::FakePcnt;

constexpr std::int16_t kLimit = 30000;

// --- CounterExtender against a simulated PCNT ---------------------------------------------------

TEST(CounterExtenderTest, TracksSmallMovementsInBothDirections) {
  FakePcnt pcnt(kLimit);
  CounterExtender extender(kLimit);

  pcnt.count(120);
  EXPECT_EQ(extender.update(pcnt.rawCount()), 120);
  pcnt.count(-200);
  EXPECT_EQ(extender.update(pcnt.rawCount()), -80);
}

TEST(CounterExtenderTest, ContinuesPastThePositiveLimit) {
  FakePcnt pcnt(kLimit);
  CounterExtender extender(kLimit);

  std::int32_t expected = 0;
  for (int step = 0; step < 20; ++step) {  // 20 x 7000 counts: several hardware resets
    pcnt.count(7000);
    expected += 7000;
    EXPECT_EQ(extender.update(pcnt.rawCount()), expected) << "step " << step;
  }
}

TEST(CounterExtenderTest, ContinuesPastTheNegativeLimit) {
  FakePcnt pcnt(kLimit);
  CounterExtender extender(kLimit);

  std::int32_t expected = 0;
  for (int step = 0; step < 20; ++step) {
    pcnt.count(-7000);
    expected -= 7000;
    EXPECT_EQ(extender.update(pcnt.rawCount()), expected) << "step " << step;
  }
}

TEST(CounterExtenderTest, MatchesRandomWalkWhenPolledFastEnough) {
  FakePcnt pcnt(kLimit);
  CounterExtender extender(kLimit);
  std::mt19937 rng(7);
  std::uniform_int_distribution<std::int32_t> move(-(kLimit / 2) + 1, (kLimit / 2) - 1);

  std::int32_t expected = 0;
  for (int i = 0; i < 2000; ++i) {
    const std::int32_t delta = move(rng);
    pcnt.count(delta);
    expected += delta;
    ASSERT_EQ(extender.update(pcnt.rawCount()), expected) << "iteration " << i;
  }
}

TEST(CounterExtenderTest, ResetRestartsFromZero) {
  FakePcnt pcnt(kLimit);
  CounterExtender extender(kLimit);
  pcnt.count(500);
  extender.update(pcnt.rawCount());

  extender.reset(pcnt.rawCount());
  pcnt.count(25);

  EXPECT_EQ(extender.update(pcnt.rawCount()), 25);
}

// --- AxisMap -------------------------------------------------------------------------------------

TEST(AxisMapTest, IdentityLeavesVectorsUnchanged) {
  constexpr AxisMap identity(1, 2, 3);
  static_assert(identity.isValid());
  const Vector3 v = identity.apply({1.0F, 2.0F, 3.0F});
  EXPECT_FLOAT_EQ(v.x, 1.0F);
  EXPECT_FLOAT_EQ(v.y, 2.0F);
  EXPECT_FLOAT_EQ(v.z, 3.0F);
}

TEST(AxisMapTest, RotationAboutZByHalfTurn) {
  constexpr AxisMap facing_backward(-1, -2, 3);
  static_assert(facing_backward.isValid());
  const Vector3 v = facing_backward.apply({1.0F, 2.0F, 3.0F});
  EXPECT_FLOAT_EQ(v.x, -1.0F);
  EXPECT_FLOAT_EQ(v.y, -2.0F);
  EXPECT_FLOAT_EQ(v.z, 3.0F);
}

TEST(AxisMapTest, SensorMountedUprightFacingForward) {
  // Board standing vertically: sensor z points forward, sensor y points up, sensor x points left.
  constexpr AxisMap upright(3, 1, 2);
  static_assert(upright.isValid());
  const Vector3 down_in_sensor{0.0F, -1.0F, 0.0F};  // sensor -y points down
  const Vector3 body = upright.apply(down_in_sensor);
  EXPECT_FLOAT_EQ(body.z, -1.0F);
  EXPECT_FLOAT_EQ(body.x, 0.0F);
  EXPECT_FLOAT_EQ(body.y, 0.0F);
}

TEST(AxisMapTest, RejectsMirrorImages) {
  static_assert(!AxisMap(1, 2, -3).isValid());  // flips handedness
  static_assert(!AxisMap(2, 1, 3).isValid());   // swaps two axes without a sign change
  EXPECT_FALSE(AxisMap(-1, 2, 3).isValid());
}

TEST(AxisMapTest, RejectsMalformedCodes) {
  EXPECT_FALSE(AxisMap(1, 1, 3).isValid());
  EXPECT_FALSE(AxisMap(0, 2, 3).isValid());
  EXPECT_FALSE(AxisMap(1, 2, 4).isValid());
  const Vector3 v = AxisMap(1, 1, 3).apply({1.0F, 2.0F, 3.0F});
  EXPECT_FLOAT_EQ(v.x + v.y + v.z, 0.0F);
}

TEST(AxisMapTest, AllProperRotationsAreAccepted) {
  int valid = 0;
  for (int x = -3; x <= 3; ++x) {
    for (int y = -3; y <= 3; ++y) {
      for (int z = -3; z <= 3; ++z) {
        valid += AxisMap(x, y, z).isValid() ? 1 : 0;
      }
    }
  }
  EXPECT_EQ(valid, 24);  // the 24 rotations of a cube
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
