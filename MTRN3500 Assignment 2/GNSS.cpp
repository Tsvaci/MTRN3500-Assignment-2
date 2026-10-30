#include "GNSS.h"

// PROGRESS-CHECK BRANCH: this module runs as a heartbeat-only stub so that TMM has a real thread
// to start, monitor and shut down. The GNSS decoding (Part 3) is implemented on the full-build branch.

GNSS::GNSS(SM_ThreadManagement^ SM_TM, SM_GPS^ SM_GPS) {
	SM_TM_ = SM_TM;
	SM_GPS_ = SM_GPS;
	northing_ = 0.0;
	easting_ = 0.0;
	height_ = 0.0;
	receivedCRC_ = 0;
	calculatedCRC_ = 0;
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
	bool flag = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		flag = (SM_TM_->shutdown & bit_GPS) != 0
			|| SM_TM_->WatchList[bitIndex(bit_TM)]->ElapsedMilliseconds > TM_ALIVE_TIMEOUT_MS;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return flag;
}

void GNSS::threadFunction() {
	Console::WriteLine("GNSS: thread started (stub - Part 3 is on the full-build branch).");

	Barrier^ barrier = nullptr;
	Monitor::Enter(SM_TM_->lockObject);
	try { barrier = SM_TM_->ThreadBarrier; }
	finally { Monitor::Exit(SM_TM_->lockObject); }
	if (barrier != nullptr) {
		barrier->SignalAndWait(BARRIER_TIMEOUT_MS);
	}

	while (!getShutdownFlag()) {
		heartbeat();
		Thread::Sleep(100);
	}
	Console::WriteLine("GNSS: thread shut down.");
}

error_state GNSS::findHeader(int% offset) {
	return error_state::SUCCESS;
}

error_state GNSS::decodeRecord(int offset) {
	return error_state::SUCCESS;
}

void GNSS::printPosition() {}

void GNSS::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_GPS;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

unsigned long GNSS::CRC32Value(int i) {
	return 0;
}

unsigned long GNSS::CalculateBlockCRC32(unsigned long ulCount, unsigned char* ucBuffer) {
	return 0;
}
