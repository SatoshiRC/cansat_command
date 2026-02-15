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
	void onReceiveFrame(const std::array<uint8_t, size> &frame){
		
		//validate frame
		//check start and stop byte
		if(*frame.begin() != START_BYTE || *frame.rbegin() != STOP_BYTE){
			return;
		}
		//check sum
		uint8_t sum = 0;
		for(auto it = frame.begin() + 1; it < frame.end() - 2; it++){
			// exclude start byte and checksum/stop bytes
			sum += *it;
		}
		if(sum != *(frame.rbegin() + 1)){
			return;
		}

		const COMMAND_ID rid = static_cast<COMMAND_ID>(frame[1]);
		std::vector<uint8_t> frameBody(frame.begin()+2, frame.end()-2);

		//check body length
		if(frameBody.size() != commandLen[static_cast<uint8_t>(rid)]){
			return;
		}

		const COMMAND_ID tid = commandHandlers[static_cast<uint8_t>(rid)]->onReceive(frameBody);
		this->transmit(tid);
	}
};

} /* namespace command */

#endif /* COMMAND_INC_COMMANDMANAGER_H_ */
