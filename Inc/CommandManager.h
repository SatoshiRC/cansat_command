/*
 * CommandHandler.h
 *
 *  Created on: Jan 5, 2026
 *      Author: OHYA Satoshi
 */

#ifndef COMMAND_INC_COMMANDMANAGER_H_
#define COMMAND_INC_COMMANDMANAGER_H_

#include "CommandHandlerBase.h"
#include "CommandHandlers.hpp"
#include <array>
#include <algorithm>

template<class ForwardIterator>
constexpr ForwardIterator getMaxElement(const ForwardIterator first, const ForwardIterator last){
    return first == (last-1) ? first : (*first < *(last-1) ? getMaxElement(first+1, last) : getMaxElement(first, last-1));
}

namespace command {

class CommandManager {
	std::array<command::Base*, (uint8_t)COMMAND_ID::Last> commandHandlers;
	static constexpr std::array<uint8_t, (uint8_t)COMMAND_ID::Last> commandLen = {
        ConnectionCheck::getDataBodyLen(),
        SensorStatus::getDataBodyLen(),
        Request::getDataBodyLen(),
        Goal::getDataBodyLen(),
        Altitude::getDataBodyLen(),
        Mode::getDataBodyLen(),
        AbsoluteNavigation::getDataBodyLen(),
        RelativeNavigation::getDataBodyLen(),
        ServoConfig_prachuteLeft::getDataBodyLen(),
        ServoConfig_prachuteRight::getDataBodyLen(),
        ServoConfig_stabilizer::getDataBodyLen(),
	};

	const uint8_t START_BYTE = 's';
	const uint8_t STOP_BYTE = 'e';


public:
	CommandManager();
	Base& operator[](COMMAND_ID id){
		return *commandHandlers[static_cast<uint8_t>(id)];
	}
  	
	std::vector<uint8_t> constructTransmitFrame(const COMMAND_ID id);
	void transmit(const COMMAND_ID id);

	template<typename _ForwardIterator>
	void receive(_ForwardIterator __first, _ForwardIterator __last){

	}

	template<size_t size>
    COMMAND_ID onReceiveFrame(const std::array<uint8_t, size> &frame){
        return onReceiveFrame(frame.begin(), frame.end());
	}
		
    COMMAND_ID onReceiveFrame(const uint8_t* __first, const uint8_t* __last){
		//validate frame
		//check start and stop byte
        if(*__first != START_BYTE || *__first != STOP_BYTE){
            return COMMAND_ID::Last;
		}
		//check sum
		uint8_t sum = 0;
        for(auto it = __first + 1; it < __last - 2; it++){
			// exclude start byte and checksum/stop bytes
			sum += *it;
		}
        if(sum != *(__last - 2)){
            return COMMAND_ID::Last;
		}

        const COMMAND_ID rid = static_cast<COMMAND_ID>(*(__first + 1));
        std::vector<uint8_t> frameBody(__first+2, __last-2);

		//check body length
		if(frameBody.size() != commandLen[static_cast<uint8_t>(rid)]){
            return COMMAND_ID::Last;
		}

		const COMMAND_ID tid = commandHandlers[static_cast<uint8_t>(rid)]->onReceive(frameBody);
        transmit(tid);
        return rid;
	}

};

} /* namespace command */

#endif /* COMMAND_INC_COMMANDMANAGER_H_ */
