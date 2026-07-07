/*
 * utest_ranger_bms_voltage.cpp
 *
 * Regression test for the Ranger Mini 3.0 BMS battery-voltage scaling bug.
 *
 * The BMS Basic frame (0x361) carries the battery voltage in units of 0.01V on
 * the Ranger Mini 2.0/3.0 (see Ranger Mini 3.0 user manual, BMS frame 0x361),
 * while the shared V2 parser decodes it with a 0.1 scale. Ranger Mini 2.0/3.0
 * therefore have to apply an additional 0.1 factor in GetCommonSensorState().
 *
 * Previously RangerMiniV3Base was aliased directly to RangerBase, which does
 * NOT apply that extra factor, so the reported voltage was 10x too high. This
 * test pins down the corrected behaviour without depending on the on-wire byte
 * order: it compares the value reported through the V3 base class against the
 * value produced by the bare V2 parser for the very same frame.
 */

#include "gtest/gtest.h"

#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"
#include "ugv_sdk/details/robot_base/ranger_base.hpp"

using namespace westonrobot;

namespace {

// Build a BMS Basic feedback frame (0x361). Field layout (per agilex_protocol_v2.h
// BmsBasicFrame): soc[0], soh[1], voltage[2-3], current[4-5], temperature[6-7].
can_frame MakeBmsBasicFrame() {
  can_frame frame{};
  frame.can_id = 0x361;
  frame.can_dlc = 8;
  frame.data[0] = 90;    // soc
  frame.data[1] = 100;   // soh
  frame.data[2] = 0x12;  // voltage (some non-zero value)
  frame.data[3] = 0xC0;
  frame.data[4] = 0x00;  // current
  frame.data[5] = 0x10;
  frame.data[6] = 0x01;  // temperature
  frame.data[7] = 0x2C;
  return frame;
}

// Test fixture exposing the protected CAN-frame entry point so we can drive the
// robot's internal state without opening an actual CAN port.
class TestRangerMiniV3 : public RangerMiniV3Base {
 public:
  void Inject(can_frame* frame) { ParseCANFrame(frame); }
};

}  // namespace

// The bare V2 parser decodes the BMS voltage with a 0.1 scale. The Ranger Mini
// V3 base class must apply an additional 0.1 factor on top of that (net 0.01V),
// matching the robot's actual 0.01V wire unit.
TEST(RangerMiniV3BmsVoltage, AppliesExtraTenthScaling) {
  can_frame frame = MakeBmsBasicFrame();

  // (1) value as decoded by the shared V2 parser
  ProtocolV2Parser parser;
  AgxMessage msg;
  ASSERT_TRUE(parser.DecodeMessage(&frame, &msg));
  ASSERT_EQ(msg.type, AgxMsgBmsBasic);
  float parser_voltage = msg.body.bms_basic_msg.voltage;
  ASSERT_GT(parser_voltage, 0.0f);

  // (2) value reported through the Ranger Mini V3 base class
  TestRangerMiniV3 robot;
  robot.Inject(&frame);
  RangerInterface* iface = &robot;
  float v3_voltage = iface->GetCommonSensorState().bms_basic_state.voltage;

  // V3 must scale down by an extra 0.1 relative to the raw parser value.
  EXPECT_NEAR(v3_voltage, parser_voltage * 0.1f, 1e-4f);

  // Guard against a regression to the old pass-through behaviour (10x too high).
  EXPECT_LT(v3_voltage, parser_voltage);
}

// Ranger Mini V3 must behave identically to V2 for BMS voltage, since both use
// the 0.01V wire unit.
TEST(RangerMiniV3BmsVoltage, MatchesV2Behaviour) {
  can_frame frame = MakeBmsBasicFrame();

  TestRangerMiniV3 v3;
  v3.Inject(&frame);
  float v3_voltage =
      static_cast<RangerInterface*>(&v3)->GetCommonSensorState().bms_basic_state.voltage;

  // reuse the same protected entry point via a local subclass
  struct TestV2 : public RangerMiniV2Base {
    void Inject(can_frame* f) { ParseCANFrame(f); }
  } v2t;
  v2t.Inject(&frame);
  float v2_voltage =
      static_cast<RangerInterface*>(&v2t)->GetCommonSensorState().bms_basic_state.voltage;

  EXPECT_NEAR(v3_voltage, v2_voltage, 1e-4f);
}
