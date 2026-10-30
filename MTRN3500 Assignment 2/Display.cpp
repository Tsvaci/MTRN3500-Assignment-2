#include "Display.h"

// PROGRESS-CHECK BRANCH: this module runs as a heartbeat-only stub so that TMM has a real thread
// to start, monitor and shut down. The MATLAB link (Part 6) is implemented on the full-build branch.

Display::Display(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser) {
	SM_TM_ = SM_TM;
	SM_Laser_ = SM_Laser;
	x_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	y_ = gcnew array<double>(STANDARD_LASER_LENGTH);
}

error_state Display::connect(String^ hostName, int portNumber) {
	return error_state::SUCCESS;
}

error_state Display::communicate() {
	return error_state::SUCCESS;
}

error_state Display::processSharedMemory() {
	return error_state::SUCCESS;
}

bool Display::getShutdownFlag() {
	bool flag = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		flag = (SM_TM_->shutdown & bit_DISPLAY) != 0
			|| SM_TM_->WatchList[bitIndex(bit_TM)]->ElapsedMilliseconds > TM_ALIVE_TIMEOUT_MS;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return flag;
}

void Display::threadFunction() {
	Console::WriteLine("Display: thread started (stub - Part 6 is on the full-build branch).");

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
	Console::WriteLine("Display: thread shut down.");
}

void Display::sendDisplayData(array<double>^ xData, array<double>^ yData) {}

void Display::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_DISPLAY;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}
