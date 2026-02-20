/*
 * Base.h
 *
 *  Created on: Jan 5, 2026
 *      Author: OHYA Satoshi
 */

#ifndef COMMAND_BASE_H_
#define COMMAND_BASE_H_

#include <cstdint>
#include <vector>
#include <functional>

namespace command {

enum class COMMAND_ID{
	ConnectionCheck = 0,
	SensorStatus,
	Request,
	Goal,
	Altitude,
	Mode,
	AbsoluteNavigationLog,
	RelativeNavigationLog,
	ServoConfig_prachuteLeft,
	ServoConfig_prachuteRight,
	ServoConfig_stabilizer,
	GPS,
	IMU,
	Last
};

class Base {
	static constexpr uint8_t dataBodyLen = 0;
	COMMAND_ID id = COMMAND_ID::Last;

protected:
	std::function<void(void)> callback = nullptr;
	void copy(const void* src, const void* dist, const uint8_t len);

public:
	Base();
	virtual COMMAND_ID onReceive(std::vector<uint8_t> &body){
		return COMMAND_ID::Last;
	};

	/* 
	 * Construct transmit frame **body**.
	 * This function shuold be called throudh CommandManager::transmit(COMMAND_ID).
	 * The return vector is data body
	 */
	virtual std::vector<uint8_t> transmit(){
		return std::vector<uint8_t>();
	}

	void setCallback(std::function<void(void)> callback){
		this->callback = callback;
	}

//	virtual uint8_t getDataBodyLen(){
//		return dataBodyLen;
//	}


};

} /* namespace command */

#endif /* COMMAND_BASE_H_ */
