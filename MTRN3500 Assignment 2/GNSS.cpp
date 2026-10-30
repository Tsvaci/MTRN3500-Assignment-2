#include "GNSS.h"

GNSS::GNSS(SM_ThreadManagement^ SM_TM, SM_GPS^ SM_GPS) {
	SM_TM_ = SM_TM;
	SM_GPS_ = SM_GPS;
}

error_state GNSS::connect(String^ hostName, int portNumber) {
	return error_state::SUCCESS;
}

error_state GNSS::communicate() {
	return error_state::SUCCESS;
}

error_state GNSS::processSharedMemory() {
	return error_state::SUCCESS;
}

bool GNSS::getShutdownFlag() {
	return true;
}

void GNSS::threadFunction() {}

error_state GNSS::findHeader(int% offset) {
	return error_state::SUCCESS;
}

error_state GNSS::decodeRecord(int offset) {
	return error_state::SUCCESS;
}

void GNSS::printPosition() {}

void GNSS::heartbeat() {}

unsigned long GNSS::CRC32Value(int i) {
	return 0;
}

unsigned long GNSS::CalculateBlockCRC32(unsigned long ulCount, unsigned char* ucBuffer) {
	return 0;
}
