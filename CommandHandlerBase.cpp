/*
 * Base.cpp
 *
 *  Created on: Jan 5, 2026
 *      Author: OHYA Satoshi
 */

#include "./Inc/CommandHandlerBase.h"

namespace command {

Base::Base() {
	// TODO Auto-generated constructor stub

}

void Base::copy(const void* src, const void* dest, const uint8_t len){
	std::copy((uint8_t*)src, (uint8_t*)src+len, (uint8_t*)dest);
}

} /* namespace command */
