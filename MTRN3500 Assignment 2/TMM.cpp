#include "TMM.h"

ThreadManagement::ThreadManagement() {}

error_state ThreadManagement::setupSharedMemory() {
	return error_state::SUCCESS;
}

error_state ThreadManagement::processSharedMemory() {
	return error_state::SUCCESS;
}

void ThreadManagement::shutdownModules() {}

bool ThreadManagement::getShutdownFlag() {
	return true;
}

void ThreadManagement::threadFunction() {}

error_state ThreadManagement::startThreads() {
	return error_state::SUCCESS;
}

error_state ThreadManagement::startModule(uint8_t moduleBit) {
	return error_state::SUCCESS;
}

uint8_t ThreadManagement::monitorHeartbeats() {
	return 0;
}

bool ThreadManagement::shutdownKeyPressed() {
	return false;
}
