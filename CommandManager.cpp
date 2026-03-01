/*
 * CommandHandler.cpp
 *
 *  Created on: Jan 5, 2026
 *      Author: OHYA Satoshi
 */

#include "./Inc/CommandManager.h"

namespace command{
	CommandManager::CommandManager() {
		// Initialize all handlers to nullptr
		commandHandlers.fill(nullptr);
	}

	std::vector<uint8_t> CommandManager::constructTransmitFrame(const COMMAND_ID id){
		// Check if handler is valid before using
		if(static_cast<uint8_t>(id) >= static_cast<uint8_t>(COMMAND_ID::Last) || commandHandlers[static_cast<uint8_t>(id)] == nullptr){
			return std::vector<uint8_t>();
		}
		auto res = commandHandlers[static_cast<uint8_t>(id)]->transmit();
		
        //check sum
		uint8_t sum = static_cast<uint8_t>(id);
		for(const auto& e : res){
			sum += e;
		}

		res.push_back(sum);
		res.push_back(STOP_BYTE);
        res.insert(res.begin(), static_cast<uint8_t>(id));
		res.insert(res.begin(), START_BYTE);

		return res;
	}

	void CommandManager::constructTransmitFrameToBuffer(const COMMAND_ID id, uint8_t* buffer, uint8_t& length){
		// Check if handler is valid before using
		if(buffer == nullptr){
			length = 0;
			return;
		}
		if(static_cast<uint8_t>(id) >= static_cast<uint8_t>(COMMAND_ID::Last) || commandHandlers[static_cast<uint8_t>(id)] == nullptr){
			length = 0;
			return;
		}
		auto res = commandHandlers[static_cast<uint8_t>(id)]->transmit();
		
        //check sum
		uint8_t sum = static_cast<uint8_t>(id);
		for(const auto& e : res){
			sum += e;
		}

		uint8_t pos = 0;
		buffer[pos++] = START_BYTE;
		buffer[pos++] = static_cast<uint8_t>(id);
		for(const auto& e : res){
			buffer[pos++] = e;
		}
		buffer[pos++] = sum;
		buffer[pos++] = STOP_BYTE;
		length = pos;
	}
	__attribute__((weak)) void CommandManager::transmit(const COMMAND_ID id){
		// Check if handler is valid before using
		if(static_cast<uint8_t>(id) >= static_cast<uint8_t>(COMMAND_ID::Last) || commandHandlers[static_cast<uint8_t>(id)] == nullptr){
			return;
		}
		auto frame = constructTransmitFrame(id);
	}

    void CommandManager::resetBuffer(){
        copyCursor = 0;
        readCursor = 0;
        rBuffer.fill(0);
    }
}
