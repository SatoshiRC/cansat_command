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
        Gps::getDataBodyLen(),
        Imu::getDataBodyLen(),
		DecentLog::getDataBodyLen(),
	};

    std::array<uint8_t, static_cast<size_t>((*getMaxElement(commandLen.begin(), commandLen.end())+4)*2)> rBuffer = {};
    uint8_t copyCursor = 0;
    uint8_t readCursor = 0;

	const uint8_t START_BYTE = 's';
	const uint8_t STOP_BYTE = 'e';
protected:
    std::array<command::Base*, (uint8_t)COMMAND_ID::Last> commandHandlers;

public:
	CommandManager();
	Base*& operator[](COMMAND_ID id){
		return commandHandlers[static_cast<uint8_t>(id)];
	}
  	
	std::vector<uint8_t> constructTransmitFrame(const COMMAND_ID id);
	void constructTransmitFrameToBuffer(const COMMAND_ID id, uint8_t* buffer, uint8_t& length);
	void transmit(const COMMAND_ID id);

	template<typename _ForwardIterator>
    COMMAND_ID receive(_ForwardIterator __first, _ForwardIterator __last){
        size_t len = std::distance(__first, __last);
        if(len == 0){
            return COMMAND_ID::Last;
        }
        if(len > rBuffer.size()){
            resetBuffer();
            return COMMAND_ID::Last;
        }

        //avoid over-flow
        if(rBuffer.size() < copyCursor + len){
            size_t copyLen = rBuffer.size() - copyCursor;
            if(copyLen > 0){
                std::copy(__first, std::next(__first, copyLen), rBuffer.begin()+copyCursor);
            }
            std::advance(__first, copyLen);
            len = std::distance(__first, __last);
            copyCursor = 0;
        }

        std::copy(__first, __last, rBuffer.begin()+copyCursor);
        copyCursor = (copyCursor + static_cast<uint16_t>(len)) % rBuffer.size();

        return COMMAND_ID::Last;
	}

    COMMAND_ID onReceiveFrame(const std::vector<uint8_t> &frame){
        if(frame.empty()) return COMMAND_ID::Last;
        return receive(frame.begin(), frame.end());
    }

	template<size_t size>
    COMMAND_ID onReceiveFrame(const std::array<uint8_t, size> &frame){
        return receive(frame.begin(), frame.end());
	}

	COMMAND_ID processReceive(){
		if(copyCursor == readCursor){
            return COMMAND_ID::Last;
		}
		int16_t reamingLen = (copyCursor - readCursor + static_cast<int16_t>(rBuffer.size())) % rBuffer.size();
		if(reamingLen == 0){
			reamingLen = rBuffer.size();
		}
        for(; reamingLen>1; reamingLen--){
			if(rBuffer[readCursor] != START_BYTE){
				readCursor = (readCursor+1)%rBuffer.size();
				continue;
			}
			uint8_t possibleId = rBuffer[(readCursor+1)%rBuffer.size()];
			if(possibleId >= static_cast<uint8_t>(COMMAND_ID::Last)){
				//There is no valid id for possibleId.
				readCursor = (readCursor+1)%rBuffer.size();
				continue;
			}
			uint8_t frameLen = commandLen[possibleId]+4;
			if(reamingLen < frameLen){
				//Wait for next receive.
				break;
			}
			if(rBuffer[(readCursor + frameLen - 1)%rBuffer.size()] != STOP_BYTE){
				readCursor = (readCursor+1)%rBuffer.size();
				continue;
			}

			std::vector<uint8_t> frame(frameLen);
			uint16_t copiedLen = 0;
			uint16_t nextCursor = (readCursor + frameLen) % rBuffer.size();
			if(readCursor < nextCursor){
				// Data is contiguous
				std::copy(rBuffer.begin()+readCursor, rBuffer.begin()+nextCursor, frame.begin());
				copiedLen = frameLen;
			} else {
				// Data wraps around buffer
				uint16_t firstPart = rBuffer.size() - readCursor;
				std::copy(rBuffer.begin()+readCursor, rBuffer.end(), frame.begin());
				copiedLen = firstPart;
				uint16_t secondPart = nextCursor;
				if(secondPart > 0){
					std::copy(rBuffer.begin(), rBuffer.begin()+secondPart, frame.begin() + copiedLen);
				}
			}
			COMMAND_ID id = onReceiveFrame(&*frame.begin(), &*frame.end());
			readCursor = nextCursor;
			reamingLen -= frameLen + 1;
            return id;
		}
        return COMMAND_ID::Last;
	}

    COMMAND_ID onReceiveFrame(const uint8_t* __first, const uint8_t* __last){
        //validate frame pointer range
        if(__first == nullptr || __last == nullptr || __first >= __last){
            return COMMAND_ID::Last;
        }
        //check minimum frame length (START + ID + DATA + CHECKSUM + STOP = at least 4 bytes)
        if(__last - __first < 4){
            return COMMAND_ID::Last;
        }
        //validate frame
        //check start and stop byte
        if(*__first != START_BYTE || *(__last-1) != STOP_BYTE){
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
        if(static_cast<uint8_t>(rid) >= static_cast<uint8_t>(COMMAND_ID::Last)){
            return COMMAND_ID::Last;
        }
        std::vector<uint8_t> frameBody(__first+2, __last-2);

        //check body length
        if(frameBody.size() != commandLen[static_cast<uint8_t>(rid)]){
            return COMMAND_ID::Last;
        }

        //check if handler is valid
        if(commandHandlers[static_cast<uint8_t>(rid)] == nullptr){
            return COMMAND_ID::Last;
        }
        const COMMAND_ID tid = commandHandlers[static_cast<uint8_t>(rid)]->onReceive(frameBody);
        transmit(tid);
        return rid;
    }

private:
    void resetBuffer();
};

} /* namespace command */

#endif /* COMMAND_INC_COMMANDMANAGER_H_ */
