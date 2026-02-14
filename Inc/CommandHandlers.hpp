/*
 * CommandHandlers.hpp
 *
 *  Created on: Jan 30, 2026
 *      Author: OHYA Satoshi
 */

#ifndef COMMAND_INC_COMMANDHANDLERS_HPP_
#define COMMAND_INC_COMMANDHANDLERS_HPP_

#include "CommandHandlerBase.h"
#include "CommandDataType.hpp"

namespace command{

class ConnectionCheck : public Base{
    static constexpr uint8_t dataBodyLen = 1;
    static constexpr COMMAND_ID id = COMMAND_ID::ConnectionCheck;

    uint8_t value = 0;
    bool isLoopback = false;
    std::function<void(uint8_t&, bool&)> update = [](uint8_t&, bool&){};
    
public:
    ConnectionCheck() = default;
    ConnectionCheck(std::function<void(uint8_t&, bool&)> update):update(update){}
    COMMAND_ID onReceive(std::vector<uint8_t> &body);
	std::vector<uint8_t> transmit();
    void setUpdate(std::function<void(uint8_t&, bool&)> func){
        update = func;
    }
};

class SensorStatus : public Base{
    static constexpr uint8_t dataBodyLen = 1;
    static constexpr COMMAND_ID id = COMMAND_ID::SensorStatus;

    CommandDataType::SensorStatus data;
    const uint8_t tofOffset = 5;
    const uint8_t cameraOffset = 4;
    const uint8_t barometerOffset = 3;
    const uint8_t magnetMeterOffset = 2;
    const uint8_t imuOffset = 1;
    const uint8_t gpsOffset = 0;
    std::function<void(CommandDataType::SensorStatus&)> callback = [](CommandDataType::SensorStatus&){ };
    std::function<void(CommandDataType::SensorStatus&)> update = [](CommandDataType::SensorStatus&){ };
    
public:
    SensorStatus() = default;
    explicit SensorStatus(const CommandDataType::SensorStatus &data):data(data){}
    SensorStatus(std::function<void(CommandDataType::SensorStatus&)> update,
                 const CommandDataType::SensorStatus &data = CommandDataType::SensorStatus()):data(data),update(update){}
    COMMAND_ID onReceive(std::vector<uint8_t> &body);
    void setCallback(std::function<void(CommandDataType::SensorStatus&)> func){
        callback = func;
    }
	std::vector<uint8_t> transmit();
    void setUpdate(std::function<void(CommandDataType::SensorStatus&)> func){
        update = func;
    }
};

class Request : public Base{
    static constexpr uint8_t dataBodyLen = 1;
    static constexpr COMMAND_ID id = COMMAND_ID::Request;

    COMMAND_ID requestID = COMMAND_ID::Last;
public:
    Request() = default;
    explicit Request(COMMAND_ID id):requestID(id){}
    COMMAND_ID onReceive(std::vector<uint8_t> &body);
	std::vector<uint8_t> transmit();

    void setRequestCommandId(COMMAND_ID id){
        requestID = id;
    }
};

class Goal : public Base{
    static constexpr uint8_t dataBodyLen = 16;
	COMMAND_ID id = COMMAND_ID::Goal;

    CommandDataType::Coordinates data;

    std::function<void(CommandDataType::Coordinates&)> callback = [](CommandDataType::Coordinates &coordinate) -> void {};
    std::function<void(CommandDataType::Coordinates&)> update = [](CommandDataType::Coordinates&){};

public:
    Goal() = default;
    explicit Goal(const CommandDataType::Coordinates &data):data(data){}
    Goal(std::function<void(CommandDataType::Coordinates&)> update,
         const CommandDataType::Coordinates &data = CommandDataType::Coordinates()):data(data),update(update){};
    COMMAND_ID onReceive(std::vector<uint8_t> &body);
    std::vector<uint8_t> transmit();
    void setCallback(std::function<void(CommandDataType::Coordinates&)> callback){
        this->callback = callback;
    }
    void setUpdate(std::function<void(CommandDataType::Coordinates&)> func){
        update = func;
    }
};

class Altitude : public Base{
    static constexpr uint8_t dataBodyLen = 10;
    COMMAND_ID id = COMMAND_ID::Altitude;

    CommandDataType::Altitude data;
    std::function<void(CommandDataType::Altitude&)> callback = [](CommandDataType::Altitude& data){};
    std::function<void(CommandDataType::Altitude&)> update = [](CommandDataType::Altitude&){ };

public:
    Altitude() = default;
    explicit Altitude(const CommandDataType::Altitude &data):data(data){}
    Altitude(std::function<void(CommandDataType::Altitude&)> update,
             const CommandDataType::Altitude &data = CommandDataType::Altitude()):data(data),update(update){}
    COMMAND_ID onReceive(std::vector<uint8_t> &body);
    std::vector<uint8_t> transmit();
    void setCallback(std::function<void(CommandDataType::Altitude&)> callback){
        this->callback = callback;
    }
    void setUpdate(std::function<void(CommandDataType::Altitude&)> func){
        update = func;
    }
};

class Mode : public Base{
    static constexpr uint8_t dataBodyLen = 1;
    COMMAND_ID id = COMMAND_ID::Mode;

    uint8_t data = 0;
    std::function<void(uint8_t)> callback = [](uint8_t mode){};
    std::function<void(uint8_t&)> update = [](uint8_t&){};

public:
    Mode() = default;
    explicit Mode(uint8_t data):data(data){}
    Mode(std::function<void(uint8_t&)> update, uint8_t data = 0):data(data),update(update){}
    COMMAND_ID onReceive(std::vector<uint8_t> &body);
    std::vector<uint8_t> transmit();
    void setCallback(std::function<void(uint8_t)> callback){
        this->callback = callback;
    }
    void setUpdate(std::function<void(uint8_t&)> func){
        update = func;
    }
};

class AbsoluteNavigation : public Base {
    static constexpr uint8_t dataBodyLen = 10;
    COMMAND_ID id = COMMAND_ID::AbsoluteNavigationLog;

    CommandDataType::AbsoluteNavigation data;
    std::function<void(CommandDataType::AbsoluteNavigation&)> callback = [](CommandDataType::AbsoluteNavigation& data){};
    std::function<void(CommandDataType::AbsoluteNavigation&)> update = [](CommandDataType::AbsoluteNavigation&){ };

public:
    AbsoluteNavigation() = default;
    explicit AbsoluteNavigation(const CommandDataType::AbsoluteNavigation &data):data(data){}
    AbsoluteNavigation(std::function<void(CommandDataType::AbsoluteNavigation&)> update,
                       const CommandDataType::AbsoluteNavigation &data = CommandDataType::AbsoluteNavigation()):data(data),update(update){}
    COMMAND_ID onReceive(std::vector<uint8_t> &body);
    std::vector<uint8_t> transmit();
    void setCallback(std::function<void(CommandDataType::AbsoluteNavigation&)> callback){
        this->callback = callback;
    }
    void setUpdate(std::function<void(CommandDataType::AbsoluteNavigation&)> func){
        update = func;
    }
};

class RelativeNavigation : public Base {
    static constexpr uint8_t dataBodyLen = 13;
    COMMAND_ID id = COMMAND_ID::RelativeNavigationLog;

    CommandDataType::RelativeNavigation data;
    std::function<void(CommandDataType::RelativeNavigation&)> callback = [](CommandDataType::RelativeNavigation& data){};
    std::function<void(CommandDataType::RelativeNavigation&)> update = [](CommandDataType::RelativeNavigation&){ };

public:
    RelativeNavigation() = default;
    explicit RelativeNavigation(const CommandDataType::RelativeNavigation &data):data(data){}
    RelativeNavigation(std::function<void(CommandDataType::RelativeNavigation&)> update,
                       const CommandDataType::RelativeNavigation &data = CommandDataType::RelativeNavigation()):data(data),update(update){}
    COMMAND_ID onReceive(std::vector<uint8_t> &body);
    std::vector<uint8_t> transmit();
    void setCallback(std::function<void(CommandDataType::RelativeNavigation&)> callback){
        this->callback = callback;
    }
    void setUpdate(std::function<void(CommandDataType::RelativeNavigation&)> func){
        update = func;
    }
};

class ServoConfig : public Base {
    static constexpr uint8_t dataBodyLen = 7;

    CommandDataType::ServoConfig data;
    std::function<void(CommandDataType::ServoConfig&)> callback = [](CommandDataType::ServoConfig& data){};
    std::function<void(CommandDataType::ServoConfig&)> update = [](CommandDataType::ServoConfig&){ };

public:
    ServoConfig() = default;
    explicit ServoConfig(const CommandDataType::ServoConfig &data):data(data){}
    ServoConfig(std::function<void(CommandDataType::ServoConfig&)> update,
                const CommandDataType::ServoConfig &data = CommandDataType::ServoConfig()):data(data),update(update){}
    COMMAND_ID onReceive(std::vector<uint8_t> &body);
    std::vector<uint8_t> transmit();
    void setCallback(std::function<void(CommandDataType::ServoConfig&)> callback){
        this->callback = callback;
    }
    void setUpdate(std::function<void(CommandDataType::ServoConfig&)> func){
        update = func;
    }
};
class ServoConfig_prachuteLeft : public ServoConfig{
    COMMAND_ID id = COMMAND_ID::ServoConfig_prachuteLeft;
};

class ServoConfig_prachuteRight : public ServoConfig {
    COMMAND_ID id = COMMAND_ID::ServoConfig_prachuteRight;
};
class ServoConfig_stabilizer : public ServoConfig {
    COMMAND_ID id = COMMAND_ID::ServoConfig_stabilizer;
};
} /*namespace command*/

#endif /* COMMAND_INC_COMMANDHANDLERS_HPP_ */
