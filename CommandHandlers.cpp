/*
 * CommandHandlers.cpp
 *
 *  Created on: Jan 30, 2026
 *      Author: OHYA Satoshi
 */

#include "./Inc/CommandHandlers.hpp"

namespace command{

COMMAND_ID ConnectionCheck::onReceive(std::vector<uint8_t> &body){
    if (body[0] >> 7 == 1){
        data = 0b0111111 & body[0];
        isLoopback = true;
    }else{
        data = 0b0111111 & body[0];
        isLoopback = false;
    }

    callback();

    return id;
}

std::vector<uint8_t> ConnectionCheck::transmit(){
    std::vector<uint8_t> res(dataBodyLen);
    res[0] = data & ((1*!isLoopback) << 7);

    return res;
}

COMMAND_ID SensorStatus::onReceive(std::vector<uint8_t> &body){
    data.tof() = body[0] & 0b1<<tofOffset;
    data.camera() = body[0] & 0b1<<cameraOffset;
    data.barometer() = body[0] & 0b1<<barometerOffset;
    data.magnetmeter() = body[0] & 0b1<<magnetMeterOffset;
    data.imu() = body[0] & 0b1<<imuOffset;
    data.gps() = body[0] & 0b1<<gpsOffset;
    callback(data);

    return COMMAND_ID::Last;
}

std::vector<uint8_t> SensorStatus::transmit(){
    update(data);
    std::vector<uint8_t> res(dataBodyLen);
    res[0] &= data.tof() << tofOffset;
    res[0] &= data.camera() << cameraOffset;
    res[0] &= data.barometer() << barometerOffset;
    res[0] &= data.magnetmeter() << magnetMeterOffset;
    res[0] &= data.imu() << imuOffset;
    res[0] &= data.gps() << gpsOffset;

    return res;
}

COMMAND_ID Request::onReceive(std::vector<uint8_t> &body){
    return static_cast<COMMAND_ID>(body[0]);
}

std::vector<uint8_t> Request::transmit(){
    std::vector<uint8_t> res(dataBodyLen);
    res[0] = static_cast<uint8_t>(requestID);

    return res;
}

COMMAND_ID Goal::onReceive(std::vector<uint8_t> &body){
    copy(body.data(), &data.latitude(), 8);
    uint8_t offset = 8;
    copy(body.data() + offset, &data.longitude(), 8);
    callback(data);
    
    return COMMAND_ID::Last;
}

std::vector<uint8_t> Goal::transmit(){
    update(data);
    std::vector<uint8_t> res(dataBodyLen);

    uint8_t offset = 0;
    uint8_t size = 8;
    copy(&data.latitude(), res.data() + offset, size);
    offset += size;
    size = 8;
    copy(&data.longitude(), res.data() + offset, size);
    
    return res;
}

COMMAND_ID Altitude::onReceive(std::vector<uint8_t> &body){
    uint8_t* it = (uint8_t*)body.data();
    std::copy(it,it+2, (uint8_t*)&data.altitude());
    it += 2;
    std::copy(it, it+4, (uint8_t*)&data.pressure());
    it += 4;
    std::copy(it, it+4, (uint8_t*)&data.temperature());

    callback(data);

    return COMMAND_ID::Last;
}

std::vector<uint8_t> Altitude::transmit(){
    update(data);
    std::vector<uint8_t> res(dataBodyLen);

    uint8_t offset = 0;
    uint8_t size = 2;
    copy(&data.altitude(), res.data() + offset, size);
    offset += size;
    size = 4;
    copy(&data.pressure(), res.data() + offset, size);
    offset += size;
    size = 4;
    copy(&data.temperature(), res.data() + offset, size);

    return res;
}

COMMAND_ID Mode::onReceive(std::vector<uint8_t> &body){
    data = body[0];
    callback(data);
    return COMMAND_ID::Last;
}

std::vector<uint8_t> Mode::transmit(){
    update(data);
    std::vector<uint8_t> res(dataBodyLen);
    res[0] = data;
    return res;
}

COMMAND_ID AbsoluteNavigation::onReceive(std::vector<uint8_t> &body){
    uint8_t offset = 0;
    uint8_t size = 3;
    copy(body.data() + offset, &data.relativePositionNorth(), size);
    if (body[offset] >> 7){
        data.relativePositionNorth() &= uint32_t(0xff)<<8*3;
    }
    offset += size;
    size = 3;
    copy(body.data() + offset, &data.relativePositionEast(), size);
    if (body[offset] >> 7){
        data.relativePositionEast() &= uint32_t(0xff)<<8*3;
    }
    offset += size;
    size = 2;
    copy(body.data() + offset, &data.headingDirection(), size);
    offset += size;
    data.leftMotorPower() = body[offset++];
    data.rightMotorPower() = body[offset++];

    callback(data);

    return COMMAND_ID::Last;
}

std::vector<uint8_t> AbsoluteNavigation::transmit(){
    update(data);
    std::vector<uint8_t> res(dataBodyLen);

    uint8_t offset = 0;
    uint8_t size = 3;
    copy(reinterpret_cast<uint8_t*>(&data.relativePositionNorth())+1, res.data() + offset, size);
    offset += size;
    size = 3;
    copy(reinterpret_cast<uint8_t*>(&data.relativePositionEast())+1, res.data() + offset, size);
    offset += size;
    size = 2;
    copy((uint8_t*)&data.headingDirection(), res.data() + offset, size);
    offset += size;
    size = 1;
    res[offset] = data.leftMotorPower();
    offset += size;
    size = 1;
    res[offset] = data.rightMotorPower();

    return res;
}

COMMAND_ID RelativeNavigation::onReceive(std::vector<uint8_t> &body){
    uint8_t offset = 0;
    uint8_t size = 3;
    copy(body.data() + offset, &data.relativePositionNorth(), size);
    if (body[offset] >> 7){
        data.relativePositionNorth() &= uint32_t(0xff)<<8*3;
    }
    offset += size;
    size = 3;
    copy(body.data() + offset, &data.relativePositionEast(), size);
    if (body[offset] >> 7){
        data.relativePositionEast() &= uint32_t(0xff)<<8*3;
    }
    offset += size;
    size = 2;
    copy(body.data() + offset, &data.headingDirection(), size);
    offset += size;
    data.leftMotorPower() = body[offset++];
    data.rightMotorPower() = body[offset++];

    data.isDetectedGoalOnCamera() = body[offset] & (0b1 << 7);
    data.isDetectedGoalOnTof() = body[offset] & (0b1 << 6);
    data.tofDistance() = (uint16_t)(body[offset]&0x1f) << 8;
    
    offset++;
    data.tofDistance() += body[offset++];
    data.goalDirection() = body[offset];

    callback(data);

    return COMMAND_ID::Last;
}

std::vector<uint8_t> RelativeNavigation::transmit(){
    update(data);
    std::vector<uint8_t> res(dataBodyLen);

    uint8_t offset = 0;
    uint8_t size = 3;
    copy(reinterpret_cast<uint8_t*>(&data.relativePositionNorth())+1, res.data() + offset, size);
    offset += size;
    size = 3;
    copy(reinterpret_cast<uint8_t*>(&data.relativePositionEast())+1, res.data() + offset, size);
    offset += size;
    size = 2;
    copy((uint8_t*)&data.headingDirection(), res.data() + offset, size);
    offset += size;
    size = 1;
    res[offset] = data.leftMotorPower();
    offset += size;
    size = 1;
    res[offset] = data.rightMotorPower();
    offset += size;

    res[offset] = data.isDetectedGoalOnCamera() << 7; 
    res[offset] &= data.isDetectedGoalOnTof() << 6; 
    res[offset] &= data.tofDistance() >> 8;
    offset++;
    res[offset] = data.tofDistance() & 0xff;
    offset++;
    res[offset] = data.goalDirection();

    return res;
}

COMMAND_ID ServoConfig::onReceive(std::vector<uint8_t> &body){
    uint8_t offset = 0;
    uint8_t size = 1;
    data.state() = (CommandDataType::ServoState)body[0];
    offset += size;
    size = 2;
    copy(body.data()+offset, &data.openCount(), size);
    offset += size;
    size = 2;
    copy(body.data()+offset, &data.centerCount(), size);
    offset += size;
    size = 2;
    copy(body.data()+offset, &data.closeCount(), size);
    offset += size;

    callback(data);
    return  COMMAND_ID::Last;
}

std::vector<uint8_t> ServoConfig::transmit(){
    update(data);
    std::vector<uint8_t> res(dataBodyLen);

    uint8_t offset = 0;
    uint8_t size = 1;
    res[offset] = static_cast<uint8_t>(data.state());
    offset += size;

    size = 2;
    copy(&data.openCount(), res.data()+offset, size);
    offset += size;

    size = 2;
    copy(&data.centerCount(), res.data()+offset, size);
    offset += size;

    size = 2;
    copy(&data.closeCount(), res.data()+offset, size);

    return res;
}

COMMAND_ID Gps::onReceive(std::vector<uint8_t> &body){
    copy(body.data(), &data.latitude(), 8);
    uint8_t offset = 8;
    copy(body.data() + offset, &data.longitude(), 8);
    offset += 8;
    copy(body.data() + offset, &data.fixStatus(), 1);
    callback(data);
    
    return COMMAND_ID::Last;
}

std::vector<uint8_t> Gps::transmit(){
    update(data);
    std::vector<uint8_t> res(dataBodyLen);

    uint8_t offset = 0;
    uint8_t size = 8;
    copy(&data.latitude(), res.data() + offset, size);
    offset += size;
    size = 8;
    copy(&data.longitude(), res.data() + offset, size);
    offset += size;
    size = 1;
    copy(&data.fixStatus(), res.data() + offset, size);
    
    return res;
}

COMMAND_ID Imu::onReceive(std::vector<uint8_t> &body){
    uint8_t offset = 0;
    uint8_t size = 4*3;
    copy(body.data()+offset, data.accel().data(), size);
    offset+=size;
    size = 4*3;
    copy(body.data()+offset, data.accel().data(), size);
    offset+=size;
    size = 4*3;
    copy(body.data()+offset, data.accel().data(), size);
    callback(data);

    return COMMAND_ID::Last;
}

std::vector<uint8_t> Imu::transmit(){
    update(data);

    std::vector<uint8_t> res(dataBodyLen);
    uint8_t offset = 0;
    uint8_t size = 4*3;
    copy(data.accel().data(), res.data()+offset, size);
    offset += size;
    size = 4*3;
    copy(data.gyro().data(), res.data()+offset, size);
    offset += size;
    size = 4*3;
    copy(data.magnet().data(), res.data()+offset, size);
}
} /*namespace command*/


