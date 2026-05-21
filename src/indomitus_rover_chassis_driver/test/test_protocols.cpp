#include <gtest/gtest.h>
#include <cstring>
#include <cmath>
#include "chassis_driver/damiao_protocol.hpp"
#include "chassis_driver/steadywin_protocol.hpp"

// ═══════════════════════════════════════════════════════════════════════════════
// Damiao protocol — frame builders
// ═══════════════════════════════════════════════════════════════════════════════

TEST(DamiaoVelocityFrame, CanIdIsBase200PlusEscId) {
    EXPECT_EQ(damiao_protocol::buildVelocityFrame(10, 0.0f).id, 0x20Au);
    EXPECT_EQ(damiao_protocol::buildVelocityFrame(16, 0.0f).id, 0x210u);
}

TEST(DamiaoVelocityFrame, DlcIsFour) {
    EXPECT_EQ(damiao_protocol::buildVelocityFrame(10, 5.0f).dlc, 4u);
}

TEST(DamiaoVelocityFrame, ValueEncodedAsLittleEndianFloat) {
    auto f = damiao_protocol::buildVelocityFrame(10, 5.0f);
    float v; std::memcpy(&v, f.data.data(), 4);
    EXPECT_NEAR(v, 5.0f, 1e-6f);
}

TEST(DamiaoVelocityFrame, NegativeVelocity) {
    auto f = damiao_protocol::buildVelocityFrame(12, -3.14f);
    float v; std::memcpy(&v, f.data.data(), 4);
    EXPECT_NEAR(v, -3.14f, 1e-5f);
}

TEST(DamiaoVelocityFrame, ZeroVelocity) {
    auto f = damiao_protocol::buildVelocityFrame(14, 0.0f);
    float v; std::memcpy(&v, f.data.data(), 4);
    EXPECT_NEAR(v, 0.0f, 1e-6f);
}

TEST(DamiaoEnableFrame, PayloadAndId) {
    auto f = damiao_protocol::buildEnableFrame(10);
    EXPECT_EQ(f.id, 10u);
    EXPECT_EQ(f.dlc, 8u);
    for (int i = 0; i < 7; i++) EXPECT_EQ(f.data[i], 0xFFu);
    EXPECT_EQ(f.data[7], 0xFCu);
}

TEST(DamiaoDisableFrame, PayloadAndId) {
    auto f = damiao_protocol::buildDisableFrame(12);
    EXPECT_EQ(f.id, 12u);
    EXPECT_EQ(f.dlc, 8u);
    for (int i = 0; i < 7; i++) EXPECT_EQ(f.data[i], 0xFFu);
    EXPECT_EQ(f.data[7], 0xFDu);
}

TEST(DamiaoSetModeFrame, VelocityMode) {
    auto f = damiao_protocol::buildSetModeFrame(10, 3);
    EXPECT_EQ(f.id, 0x7FFu);
    EXPECT_EQ(f.dlc, 8u);
    EXPECT_EQ(f.data[0], 10u);   // esc_id low byte
    EXPECT_EQ(f.data[1], 0u);    // esc_id high byte
    EXPECT_EQ(f.data[2], 0x55u); // write sentinel
    EXPECT_EQ(f.data[3], 10u);   // register 10 = CTRL_MODE
    EXPECT_EQ(f.data[4], 3u);    // mode 3 = velocity
}

// ═══════════════════════════════════════════════════════════════════════════════
// Damiao protocol — feedback parser
// ═══════════════════════════════════════════════════════════════════════════════

// Build a valid MIT feedback frame for a given motor ID and values
static std::array<uint8_t, 8> makeDamiaoFeedback(
    uint8_t esc_id, uint8_t err,
    float pos, float vel, float tor,
    float pmax, float vmax, float tmax,
    uint8_t t_mos = 35, uint8_t t_rotor = 40)
{
    std::array<uint8_t, 8> d{};
    d[0] = static_cast<uint8_t>((err << 4) | (esc_id & 0x0Fu));
    uint16_t p_int = damiao_protocol::floatToUint(pos, -pmax, pmax, 16);
    uint16_t v_int = damiao_protocol::floatToUint(vel, -vmax, vmax, 12);
    uint16_t t_int = damiao_protocol::floatToUint(tor, -tmax, tmax, 12);
    d[1] = (p_int >> 8) & 0xFFu;
    d[2] =  p_int       & 0xFFu;
    d[3] = (v_int >> 4) & 0xFFu;
    d[4] = static_cast<uint8_t>(((v_int & 0x0Fu) << 4) | ((t_int >> 8) & 0x0Fu));
    d[5] =  t_int       & 0xFFu;
    d[6] = t_mos;
    d[7] = t_rotor;
    return d;
}

TEST(DamiaoFeedback, ParsesCorrectMotor) {
    auto data = makeDamiaoFeedback(10, 1, 0.0f, 0.0f, 0.0f, 12.5f, 50.0f, 20.0f, 35, 40);
    damiao_protocol::MotorState s;
    EXPECT_TRUE(damiao_protocol::parseFeedback(data, 8, 10, 12.5f, 50.0f, 20.0f, s));
    EXPECT_TRUE(s.valid);
    EXPECT_EQ(s.err, 1u);
    EXPECT_EQ(s.t_mos, 35u);
    EXPECT_EQ(s.t_rotor, 40u);
}

TEST(DamiaoFeedback, PositionRoundTrip) {
    auto data = makeDamiaoFeedback(10, 1, 3.14f, 0.0f, 0.0f, 12.5f, 50.0f, 20.0f);
    damiao_protocol::MotorState s;
    damiao_protocol::parseFeedback(data, 8, 10, 12.5f, 50.0f, 20.0f, s);
    EXPECT_NEAR(s.pos, 3.14f, 0.01f);
}

TEST(DamiaoFeedback, VelocityRoundTrip) {
    auto data = makeDamiaoFeedback(10, 1, 0.0f, 12.5f, 0.0f, 12.5f, 50.0f, 20.0f);
    damiao_protocol::MotorState s;
    damiao_protocol::parseFeedback(data, 8, 10, 12.5f, 50.0f, 20.0f, s);
    EXPECT_NEAR(s.vel, 12.5f, 0.1f);
}

TEST(DamiaoFeedback, TorqueRoundTrip) {
    auto data = makeDamiaoFeedback(12, 1, 0.0f, 0.0f, 5.0f, 12.5f, 50.0f, 20.0f);
    damiao_protocol::MotorState s;
    damiao_protocol::parseFeedback(data, 8, 12, 12.5f, 50.0f, 20.0f, s);
    EXPECT_NEAR(s.tor, 5.0f, 0.05f);
}

TEST(DamiaoFeedback, RejectsWrongMotorId) {
    auto data = makeDamiaoFeedback(10, 1, 0.0f, 0.0f, 0.0f, 12.5f, 50.0f, 20.0f);
    damiao_protocol::MotorState s;
    EXPECT_FALSE(damiao_protocol::parseFeedback(data, 8, 12, 12.5f, 50.0f, 20.0f, s));
    EXPECT_FALSE(s.valid);
}

TEST(DamiaoFeedback, RejectsShortDlc) {
    auto data = makeDamiaoFeedback(10, 1, 0.0f, 0.0f, 0.0f, 12.5f, 50.0f, 20.0f);
    damiao_protocol::MotorState s;
    EXPECT_FALSE(damiao_protocol::parseFeedback(data, 7, 10, 12.5f, 50.0f, 20.0f, s));
}

// ═══════════════════════════════════════════════════════════════════════════════
// Steadywin protocol — frame builders
// ═══════════════════════════════════════════════════════════════════════════════

TEST(SteadywinAbsPosition, ZeroAngle) {
    auto f = steadywin_protocol::buildAbsPositionFrame(11, 0.0f);
    EXPECT_EQ(f.id, 11u);
    EXPECT_EQ(f.dlc, 5u);
    EXPECT_EQ(f.data[0], 0xC2u);
    int32_t counts; std::memcpy(&counts, f.data.data() + 1, 4);
    EXPECT_EQ(counts, 0);
}

TEST(SteadywinAbsPosition, HalfRevolution) {
    // π rad = 8192 counts  (16384 / 2)
    auto f = steadywin_protocol::buildAbsPositionFrame(11, static_cast<float>(M_PI));
    int32_t counts; std::memcpy(&counts, f.data.data() + 1, 4);
    EXPECT_EQ(counts, 8192);
}

TEST(SteadywinAbsPosition, QuarterRevolutionNegative) {
    // -π/2 rad = -4096 counts
    auto f = steadywin_protocol::buildAbsPositionFrame(13, -static_cast<float>(M_PI) / 2.0f);
    int32_t counts; std::memcpy(&counts, f.data.data() + 1, 4);
    EXPECT_EQ(counts, -4096);
}

TEST(SteadywinAbsPosition, OneRevolution) {
    // 2π rad = 16384 counts
    auto f = steadywin_protocol::buildAbsPositionFrame(15, 2.0f * static_cast<float>(M_PI));
    int32_t counts; std::memcpy(&counts, f.data.data() + 1, 4);
    EXPECT_EQ(counts, 16384);
}

TEST(SteadywinAbsPosition, SmallAngle45deg) {
    // 45° = π/4 rad = 2048 counts
    auto f = steadywin_protocol::buildAbsPositionFrame(17, static_cast<float>(M_PI) / 4.0f);
    int32_t counts; std::memcpy(&counts, f.data.data() + 1, 4);
    EXPECT_EQ(counts, 2048);
}

TEST(SteadywinDisable, FrameFormat) {
    auto f = steadywin_protocol::buildDisableFrame(13);
    EXPECT_EQ(f.id, 13u);
    EXPECT_EQ(f.dlc, 1u);
    EXPECT_EQ(f.data[0], 0xCFu);
}

TEST(SteadywinClearFault, FrameFormat) {
    auto f = steadywin_protocol::buildClearFaultFrame(15);
    EXPECT_EQ(f.id, 15u);
    EXPECT_EQ(f.dlc, 1u);
    EXPECT_EQ(f.data[0], 0xAFu);
}

TEST(SteadywinStatusQuery, FrameFormat) {
    auto f = steadywin_protocol::buildStatusQueryFrame(17);
    EXPECT_EQ(f.id, 17u);
    EXPECT_EQ(f.dlc, 1u);
    EXPECT_EQ(f.data[0], 0xAEu);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Steadywin protocol — status feedback parser
// ═══════════════════════════════════════════════════════════════════════════════

TEST(SteadywinStatusFeedback, ParsesVoltageCurrentTempModeFault) {
    // voltage=24.40V (2440=0x0988 LE), current=0.01A (1=0x0001 LE), temp=38, mode=4, fault=0
    std::array<uint8_t, 8> d = {0xAE, 0x88, 0x09, 0x01, 0x00, 38, 4, 0};
    steadywin_protocol::MotorState s;
    EXPECT_TRUE(steadywin_protocol::parseStatusResponse(d, 8, s));
    EXPECT_TRUE(s.diag_valid);
    EXPECT_NEAR(s.voltage, 24.40f, 0.01f);
    EXPECT_NEAR(s.bus_current, 0.01f, 0.001f);
    EXPECT_EQ(s.temperature, 38u);
    EXPECT_EQ(s.mode, 4u);
    EXPECT_EQ(s.fault_code, 0u);
}

TEST(SteadywinStatusFeedback, ParsesFaultCode) {
    // fault_code = 0x05 = voltage fault (Bit0) + temperature fault (Bit2)
    std::array<uint8_t, 8> d = {0xAE, 0x00, 0x00, 0x00, 0x00, 90, 3, 0x05};
    steadywin_protocol::MotorState s;
    EXPECT_TRUE(steadywin_protocol::parseStatusResponse(d, 8, s));
    EXPECT_EQ(s.fault_code, 0x05u);
    EXPECT_EQ(s.temperature, 90u);
}

TEST(SteadywinStatusFeedback, RejectsWrongCommand) {
    std::array<uint8_t, 8> d = {0xC2, 0, 0, 0, 0, 0, 0, 0};
    steadywin_protocol::MotorState s;
    EXPECT_FALSE(steadywin_protocol::parseStatusResponse(d, 8, s));
    EXPECT_FALSE(s.diag_valid);
}

TEST(SteadywinStatusFeedback, RejectsShortDlc) {
    std::array<uint8_t, 8> d = {0xAE, 0, 0, 0, 0, 0, 0, 0};
    steadywin_protocol::MotorState s;
    EXPECT_FALSE(steadywin_protocol::parseStatusResponse(d, 7, s));
}

// ═══════════════════════════════════════════════════════════════════════════════
// Steadywin protocol — position feedback parser
// ═══════════════════════════════════════════════════════════════════════════════

// data[3-6] = multi-turn int32 LE counts; 16384 counts = 2π rad
static std::array<uint8_t, 8> makeSteadywinPosResponse(uint8_t cmd, int32_t multi_counts) {
    std::array<uint8_t, 8> d{};
    d[0] = cmd;
    // data[1-2] single-turn (not used in parsing)
    std::memcpy(d.data() + 3, &multi_counts, 4);
    return d;
}

TEST(SteadywinPositionFeedback, ZeroPosition) {
    auto d = makeSteadywinPosResponse(0xC2, 0);
    steadywin_protocol::MotorState s;
    EXPECT_TRUE(steadywin_protocol::parsePositionResponse(d, 7, s));
    EXPECT_TRUE(s.pos_valid);
    EXPECT_NEAR(s.pos_rad, 0.0f, 1e-4f);
}

TEST(SteadywinPositionFeedback, OneFullRevolution) {
    // 16384 counts = 2π rad
    auto d = makeSteadywinPosResponse(0xC2, 16384);
    steadywin_protocol::MotorState s;
    steadywin_protocol::parsePositionResponse(d, 7, s);
    EXPECT_NEAR(s.pos_rad, 2.0f * static_cast<float>(M_PI), 0.001f);
}

TEST(SteadywinPositionFeedback, NegativePosition) {
    // -8192 counts = -π rad
    auto d = makeSteadywinPosResponse(0xA3, -8192);
    steadywin_protocol::MotorState s;
    steadywin_protocol::parsePositionResponse(d, 7, s);
    EXPECT_NEAR(s.pos_rad, -static_cast<float>(M_PI), 0.001f);
}

TEST(SteadywinPositionFeedback, AcceptsA3Command) {
    auto d = makeSteadywinPosResponse(0xA3, 4096);
    steadywin_protocol::MotorState s;
    EXPECT_TRUE(steadywin_protocol::parsePositionResponse(d, 7, s));
}

TEST(SteadywinPositionFeedback, AcceptsC3Command) {
    auto d = makeSteadywinPosResponse(0xC3, 0);
    steadywin_protocol::MotorState s;
    EXPECT_TRUE(steadywin_protocol::parsePositionResponse(d, 7, s));
}

TEST(SteadywinPositionFeedback, RejectsAECommand) {
    auto d = makeSteadywinPosResponse(0xAE, 0);
    steadywin_protocol::MotorState s;
    EXPECT_FALSE(steadywin_protocol::parsePositionResponse(d, 7, s));
}

TEST(SteadywinPositionFeedback, RejectsShortDlc) {
    auto d = makeSteadywinPosResponse(0xC2, 0);
    steadywin_protocol::MotorState s;
    EXPECT_FALSE(steadywin_protocol::parsePositionResponse(d, 6, s));
}

// ═══════════════════════════════════════════════════════════════════════════════
// Fixed-point helpers
// ═══════════════════════════════════════════════════════════════════════════════

TEST(FixedPoint, FloatToUintAndBack_midrange) {
    float original = 3.0f;
    uint16_t encoded = damiao_protocol::floatToUint(original, -12.5f, 12.5f, 16);
    float decoded    = damiao_protocol::uintToFloat(encoded, -12.5f, 12.5f, 16);
    EXPECT_NEAR(decoded, original, 0.001f);
}

TEST(FixedPoint, FloatToUintAndBack_zero) {
    uint16_t encoded = damiao_protocol::floatToUint(0.0f, -50.0f, 50.0f, 12);
    float decoded    = damiao_protocol::uintToFloat(encoded, -50.0f, 50.0f, 12);
    EXPECT_NEAR(decoded, 0.0f, 0.05f);
}

TEST(FixedPoint, ClampsAboveMax) {
    uint16_t hi = damiao_protocol::floatToUint(999.0f, -12.5f, 12.5f, 16);
    uint16_t max_val = (1 << 16) - 1;
    EXPECT_EQ(hi, max_val);
}

TEST(FixedPoint, ClampsBelowMin) {
    uint16_t lo = damiao_protocol::floatToUint(-999.0f, -12.5f, 12.5f, 16);
    EXPECT_EQ(lo, 0u);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
