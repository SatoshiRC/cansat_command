#include "../Inc/CommandHandlers.hpp"

#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace command;

namespace {

template <typename T>
std::array<uint8_t, sizeof(T)> toBytes(const T &value) {
    std::array<uint8_t, sizeof(T)> out{};
    std::memcpy(out.data(), &value, sizeof(T));
    return out;
}

std::array<uint8_t, 3> encodeSigned24Host(int32_t value) {
    std::array<uint8_t, 3> out{};
    const auto *raw = reinterpret_cast<const uint8_t *>(&value);
    out[0] = raw[1];
    out[1] = raw[2];
    out[2] = raw[3];
    return out;
}

void expectBytes(const uint8_t *actual, const uint8_t *expected, size_t len, const char *message) {
    if (std::memcmp(actual, expected, len) != 0) {
        throw std::runtime_error(message);
    }
}

void testGoalTransmit() {
    CommandDataType::Coordinates coordinates;
    coordinates.latitude() = 35.689487;
    coordinates.longitude() = 139.691711;

    Goal goal(coordinates);
    const auto payload = goal.transmit();
    if (payload.size() != 16) {
        throw std::runtime_error("Goal payload length mismatch");
    }

    const auto expectedLat = toBytes(coordinates.latitude());
    const auto expectedLon = toBytes(coordinates.longitude());

    expectBytes(payload.data(), expectedLat.data(), expectedLat.size(), "Goal latitude encoding mismatch");
    expectBytes(payload.data() + 8, expectedLon.data(), expectedLon.size(), "Goal longitude encoding mismatch");
}

void testAltitudeTransmit() {
    CommandDataType::Altitude altitude;
    altitude.altitude() = 1234;
    altitude.pressure() = 1013.25f;
    altitude.temperature() = 24.5f;

    Altitude handler(altitude);
    const auto payload = handler.transmit();
    if (payload.size() != 10) {
        throw std::runtime_error("Altitude payload length mismatch");
    }

    const auto expectedAlt = toBytes(altitude.altitude());
    const auto expectedPressure = toBytes(altitude.pressure());
    const auto expectedTemp = toBytes(altitude.temperature());

    size_t offset = 0;
    expectBytes(payload.data() + offset, expectedAlt.data(), expectedAlt.size(), "Altitude encoding mismatch");
    offset += expectedAlt.size();
    expectBytes(payload.data() + offset, expectedPressure.data(), expectedPressure.size(), "Pressure encoding mismatch");
    offset += expectedPressure.size();
    expectBytes(payload.data() + offset, expectedTemp.data(), expectedTemp.size(), "Temperature encoding mismatch");
}

void testModeTransmit() {
    uint8_t currentMode = 0x5A;
    Mode mode(currentMode);

    const auto payload = mode.transmit();
    if (payload.size() != 1) {
        throw std::runtime_error("Mode payload length mismatch");
    }
    if (payload[0] != currentMode) {
        throw std::runtime_error("Mode value mismatch");
    }
}

void testAbsoluteNavigationTransmit() {
    CommandDataType::AbsoluteNavigation nav;
    nav.relativePositionNorth() = 0x00112233;
    nav.relativePositionEast() = -0x00012345;
    nav.headingDirection() = -123;
    nav.leftMotorPower() = 90;
    nav.rightMotorPower() = -45;

    AbsoluteNavigation handler(nav);
    const auto payload = handler.transmit();
    if (payload.size() != 10) {
        throw std::runtime_error("AbsoluteNavigation payload length mismatch");
    }

    size_t offset = 0;
    const auto north = encodeSigned24Host(nav.relativePositionNorth());
    expectBytes(payload.data() + offset, north.data(), north.size(), "Relative north encoding mismatch");
    offset += north.size();

    const auto east = encodeSigned24Host(nav.relativePositionEast());
    expectBytes(payload.data() + offset, east.data(), east.size(), "Relative east encoding mismatch");
    offset += east.size();

    const auto heading = toBytes(nav.headingDirection());
    expectBytes(payload.data() + offset, heading.data(), heading.size(), "Heading encoding mismatch");
    offset += heading.size();

    if (payload[offset++] != static_cast<uint8_t>(nav.leftMotorPower())) {
        throw std::runtime_error("Left motor encoding mismatch");
    }
    if (payload[offset++] != static_cast<uint8_t>(nav.rightMotorPower())) {
        throw std::runtime_error("Right motor encoding mismatch");
    }
}

void testRelativeNavigationTransmit() {
    CommandDataType::RelativeNavigation nav;
    nav.relativePositionNorth() = -0x00045678;
    nav.relativePositionEast() = 0x0000FEDC;
    nav.headingDirection() = 45;
    nav.leftMotorPower() = -10;
    nav.rightMotorPower() = 10;
    nav.isDetectedGoalOnCamera() = true;
    nav.isDetectedGoalOnTof() = true;
    nav.tofDistance() = 0x01FF;
    nav.goalDirection() = -90;

    RelativeNavigation handler(nav);
    const auto payload = handler.transmit();
    if (payload.size() != 13) {
        throw std::runtime_error("RelativeNavigation payload length mismatch");
    }

    size_t offset = 0;
    const auto north = encodeSigned24Host(nav.relativePositionNorth());
    expectBytes(payload.data() + offset, north.data(), north.size(), "Relative north encoding mismatch");
    offset += north.size();

    const auto east = encodeSigned24Host(nav.relativePositionEast());
    expectBytes(payload.data() + offset, east.data(), east.size(), "Relative east encoding mismatch");
    offset += east.size();

    const auto heading = toBytes(nav.headingDirection());
    expectBytes(payload.data() + offset, heading.data(), heading.size(), "Heading encoding mismatch");
    offset += heading.size();

    if (payload[offset++] != static_cast<uint8_t>(nav.leftMotorPower())) {
        throw std::runtime_error("Left motor encoding mismatch");
    }
    if (payload[offset++] != static_cast<uint8_t>(nav.rightMotorPower())) {
        throw std::runtime_error("Right motor encoding mismatch");
    }

    uint8_t expectedStatus = (nav.isDetectedGoalOnCamera() ? 1 : 0) << 7;
    expectedStatus &= (nav.isDetectedGoalOnTof() ? 1 : 0) << 6;
    expectedStatus &= static_cast<uint8_t>(nav.tofDistance() >> 8);
    if (payload[offset++] != expectedStatus) {
        throw std::runtime_error("Status byte encoding mismatch");
    }
    if (payload[offset++] != static_cast<uint8_t>(nav.tofDistance() & 0xFF)) {
        throw std::runtime_error("ToF low byte mismatch");
    }
    if (payload[offset++] != static_cast<uint8_t>(nav.goalDirection())) {
        throw std::runtime_error("Goal direction mismatch");
    }
}

void testSensorStatusTransmit() {
    CommandDataType::SensorStatus status;
    status.tof() = true;
    status.camera() = true;
    status.barometer() = true;
    status.magnetmeter() = true;
    status.imu() = true;
    status.gps() = true;

    SensorStatus handler(status);
    const auto payload = handler.transmit();
    if (payload.size() != 1) {
        throw std::runtime_error("SensorStatus payload length mismatch");
    }
    if (payload[0] != 0) {
        throw std::runtime_error("SensorStatus expected zero encoding");
    }
}

void testServoConfigTransmit() {
    CommandDataType::ServoConfig config;
    config.state() = CommandDataType::ServoState::Center;
    config.openCount() = 0x1234;
    config.centerCount() = 0x3456;
    config.closeCount() = 0x789A;

    ServoConfig handler(config);
    const auto payload = handler.transmit();
    if (payload.size() != 7) {
        throw std::runtime_error("ServoConfig payload length mismatch");
    }

    size_t offset = 0;
    if (payload[offset++] != static_cast<uint8_t>(config.state())) {
        throw std::runtime_error("ServoConfig state encoding mismatch");
    }

    const auto open = toBytes(config.openCount());
    expectBytes(payload.data() + offset, open.data(), open.size(), "ServoConfig open count mismatch");
    offset += open.size();

    const auto center = toBytes(config.centerCount());
    expectBytes(payload.data() + offset, center.data(), center.size(), "ServoConfig center count mismatch");
    offset += center.size();

    const auto close = toBytes(config.closeCount());
    expectBytes(payload.data() + offset, close.data(), close.size(), "ServoConfig close count mismatch");
}

using TestFunc = void (*)();

const std::vector<std::pair<const char *, TestFunc>> tests = {
    {"Goal transmit", testGoalTransmit},
    {"Altitude transmit", testAltitudeTransmit},
    {"Mode transmit", testModeTransmit},
    {"Absolute navigation transmit", testAbsoluteNavigationTransmit},
    {"Relative navigation transmit", testRelativeNavigationTransmit},
    {"Sensor status transmit", testSensorStatusTransmit},
    {"Servo config transmit", testServoConfigTransmit}
};

} // namespace

int main() {
    bool success = true;
    for (const auto &test : tests) {
        try {
            test.second();
            std::cout << "[PASS] " << test.first << '\n';
        } catch (const std::exception &ex) {
            success = false;
            std::cerr << "[FAIL] " << test.first << ": " << ex.what() << '\n';
        }
    }

    return success ? 0 : 1;
}
