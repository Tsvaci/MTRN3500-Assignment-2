#include "GNSS.h"

// PROGRESS-CHECK BRANCH: this module runs as a heartbeat-only stub so that TMM has a real thread
// to start, monitor and shut down. The GNSS decoding (Part 3) will be implemented on the full-build branch.

static const int STUB_LOOP_PERIOD_MS = 100;	// How often the stub reports "alive"

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

// Stop when TMM sets this module's shutdown bit, or when TMM itself has stopped running.
bool GNSS::getShutdownFlag() {
	bool shutdownRequested = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdownRequested = (SM_TM_->shutdown & bit_GPS) != 0;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdownRequested || tmmHasStopped();
}

void GNSS::threadFunction() {
	Console::WriteLine("GNSS: thread started (stub - Part 3 comes in the full build).");

	waitForStartBarrier();

	// Stub loop: nothing to do yet except tell TMM this thread is still alive.
	while (!getShutdownFlag()) {
		heartbeat();
		Thread::Sleep(STUB_LOOP_PERIOD_MS);
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

// Tell TMM this thread is alive: TMM watches for this bit and clears it each time it sees it.
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
