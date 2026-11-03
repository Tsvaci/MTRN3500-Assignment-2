#include "Display.h"

// Progress-check stub: heartbeat only.

static const int STUB_LOOP_PERIOD_MS = 100;

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
	bool shutdownRequested = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdownRequested = (SM_TM_->shutdown & bit_DISPLAY) != 0;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdownRequested || tmmHasStopped();
}

void Display::threadFunction() {
	Console::WriteLine("Display: thread started (stub - Part 6 comes in the full build).");

	waitForStartBarrier();

	while (!getShutdownFlag()) {
		heartbeat();
		Thread::Sleep(STUB_LOOP_PERIOD_MS);
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
