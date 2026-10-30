#include "LiDAR.h"

LiDAR::LiDAR(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser) {
	SM_TM_ = SM_TM;
	SM_Laser_ = SM_Laser;
}

error_state LiDAR::connect(String^ hostName, int portNumber) {
	return error_state::SUCCESS;
}

error_state LiDAR::communicate() {
	return error_state::SUCCESS;
}

error_state LiDAR::processSharedMemory() {
	return error_state::SUCCESS;
}

bool LiDAR::getShutdownFlag() {
	return true;
}

void LiDAR::threadFunction() {}

error_state LiDAR::authenticate() {
	return error_state::SUCCESS;
}

error_state LiDAR::requestScan() {
	return error_state::SUCCESS;
}

error_state LiDAR::readTelegram(String^% telegram) {
	return error_state::SUCCESS;
}

error_state LiDAR::parseScan(String^ telegram) {
	return error_state::SUCCESS;
}

void LiDAR::printScan() {}

void LiDAR::heartbeat() {}
