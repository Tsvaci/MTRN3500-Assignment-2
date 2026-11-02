#include "Display.h"

// Placeholder for the progress check. For now this thread only tells TMM it is running,
// so TMM has a real thread to start, watch and stop. Sending scans to MATLAB comes in the full build.

static const int STUB_LOOP_PERIOD_MS = 100;	// How often the stub reports "alive"

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

// Stop when TMM sets this module's shutdown bit, or when TMM itself has stopped running.
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

	// Stub loop: nothing to do yet except tell TMM this thread is still alive.
	while (!getShutdownFlag()) {
		heartbeat();
		Thread::Sleep(STUB_LOOP_PERIOD_MS);
	}
	Console::WriteLine("Display: thread shut down.");
}

void Display::sendDisplayData(array<double>^ xData, array<double>^ yData) {}

// Tell TMM this thread is alive: TMM watches for this bit and clears it each time it sees it.
void Display::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_DISPLAY;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}
