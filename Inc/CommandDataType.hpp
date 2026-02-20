#ifndef DATA_TYPE_HPP
#define DATA_TYPE_HPP

#include <array>
#include <cstdint>

namespace CommandDataType {

class SensorStatus {
    bool _tof = false;
    bool _camera = false;
    bool _barometer = false;
    bool _magnetmeter = false;
    bool _imu = false;
    bool _gps = false;

public:
    bool& tof() { return _tof; }
    const bool& tof() const { return _tof; }

    bool& camera() { return _camera; }
    const bool& camera() const { return _camera; }

    bool& barometer() { return _barometer; }
    const bool& barometer() const { return _barometer; }

    bool& magnetmeter() { return _magnetmeter; }
    const bool& magnetmeter() const { return _magnetmeter; }

    bool& imu() { return _imu; }
    const bool& imu() const { return _imu; }

    bool& gps() { return _gps; }
    const bool& gps() const { return _gps; }
};

class Coordinates {
    double _latitude = 0.0;
    double _longitude = 0.0;

public:
    double& longitude() { return _longitude; }
    const double& longitude() const { return _longitude; }

    double& latitude() { return _latitude; }
    const double& latitude() const { return _latitude; }
};

class Altitude {
    int16_t _altitude = 0;
    float _pressure = 0;
    float _temperature = 0;

public:
    int16_t& altitude() { return _altitude; }
    const int16_t& altitude() const { return _altitude; }

    float& pressure() { return _pressure; }
    const float& pressure() const { return _pressure; }

    float& temperature() { return _temperature; }
    const float& temperature() const { return _temperature; }
};

class AbsoluteNavigation {
    int32_t _relativePositionNorth = 0;
    int32_t _relativePositionEast = 0;
    int16_t _headingDirection = 0;
    int8_t _leftMotorPower = 0;
    int8_t _rightMotorPower = 0;

public:
    int32_t& relativePositionNorth() { return _relativePositionNorth; }
    const int32_t& relativePositionNorth() const { return _relativePositionNorth; }

    int32_t& relativePositionEast() { return _relativePositionEast; }
    const int32_t& relativePositionEast() const { return _relativePositionEast; }

    int16_t& headingDirection() { return _headingDirection; }
    const int16_t& headingDirection() const { return _headingDirection; }

    int8_t& leftMotorPower() { return _leftMotorPower; }
    const int8_t& leftMotorPower() const { return _leftMotorPower; }

    int8_t& rightMotorPower() { return _rightMotorPower; }
    const int8_t& rightMotorPower() const { return _rightMotorPower; }
};

class RelativeNavigation : public AbsoluteNavigation {
    bool _isDetectedGoalOnCamera = false;
    bool _isDetectedGoalOnTof = false;
    int16_t _tofDistance = 0;
    int8_t _goalDirection = 0;

public:
    bool& isDetectedGoalOnCamera() { return _isDetectedGoalOnCamera; }
    const bool& isDetectedGoalOnCamera() const { return _isDetectedGoalOnCamera; }

    bool& isDetectedGoalOnTof() { return _isDetectedGoalOnTof; }
    const bool& isDetectedGoalOnTof() const { return _isDetectedGoalOnTof; }

    int16_t& tofDistance() { return _tofDistance; }
    const int16_t& tofDistance() const { return _tofDistance; }

    int8_t& goalDirection() { return _goalDirection; }
    const int8_t& goalDirection() const { return _goalDirection; }
};

enum class ServoState{
    Disabled = 0,
    Open,
    Center,
    Close
};

class ServoConfig {
    uint16_t _openCount = 0;
    uint16_t _closeCount = 0;
    uint16_t _centerCount = 0;
    ServoState _state = ServoState::Disabled;

public:
    uint16_t& openCount() { return _openCount; }
    const uint16_t& openCount() const { return _openCount; }

    uint16_t& closeCount() { return _closeCount; }
    const uint16_t& closeCount() const { return _closeCount; }

    uint16_t& centerCount() { return _centerCount; }
    const uint16_t& centerCount() const { return _centerCount; }

    ServoState& state() { return _state; }
    const ServoState& state() const { return _state; }
};

class GPS {
    double _latitude = 0;
    double _longitude = 0;
    uint8_t _fixStatus = 0;

public:
    double& latitude() { return _latitude; }
    const double& latitude() const { return _latitude; }

    double& longitude() { return _longitude; }
    const double& longitude() const { return _longitude; }

    uint8_t& fixStatus() { return _fixStatus; }
    const uint8_t& fixStatus() const { return _fixStatus; }

};

class IMU {
    std::array<float, 3> _accel = {};
    std::array<float, 3> _gyro = {};
    std::array<float, 3> _magnet = {};
public:
    std::array<float, 3>& accel() { return _accel; }
    const std::array<float, 3>& accel() const { return _accel; }

    std::array<float, 3>& gyro() { return _gyro; }
    const std::array<float, 3>& gyro() const { return _gyro; }

    std::array<float, 3>& magnet() { return _magnet; }
    const std::array<float, 3>& magnet() const { return _magnet; }
};

} // namespace DataType
#endif /* DATA_TYPE_HPP */