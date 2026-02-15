/*
 * CommandHandler.cpp
 *
 *  Created on: Jan 5, 2026
 *      Author: OHYA Satoshi
 */

#include "./Inc/CommandManager.h"

namespace command{
	CommandManager::CommandManager() {
		// TODO Auto-generated constructor stub

	}

	std::vector<uint8_t> CommandManager::constructTransmitFrame(const COMMAND_ID id){
		auto res = commandHandlers[static_cast<uint8_t>(id)]->transmit();
		
		//check sum
		uint8_t sum = static_cast<uint8_t>(id);
		for(const auto& e : res){
			sum += e;
		}

		res.push_back(sum);
		res.push_back(STOP_BYTE);
		res.insert(res.begin(), START_BYTE);

		return res;
	}
	__attribute__((weak)) void CommandManager::transmit(const COMMAND_ID id){
		auto frame = constructTransmitFrame(id);
	}

    void CommandManager::resetBuffer(){
        copyCursor = 0;
        readCursor = 0;
        rBuffer.fill(0);
    }
}
