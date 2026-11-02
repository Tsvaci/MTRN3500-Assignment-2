#include "VC.h"

// Placeholder for the progress check. For now this thread only tells TMM it is running,
// so TMM has a real thread to start, watch and stop. Driving the vehicle comes in the full build.

static const int STUB_LOOP_PERIOD_MS = 100;	// How often the stub reports "alive"

VehicleControl::VehicleControl(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC, SM_CrashAvoidance^ SM_CA) {
	SM_TM_ = SM_TM;
	SM_VC_ = SM_VC;
	SM_CA_ = SM_CA;
	speed_ = 0.0;
	steering_ = 0.0;
	inhibit_ = false;
	watchdog_ = 0;
}

error_state VehicleControl::connect(String^ hostName, int portNumber) {
	return error_state::SUCCESS;
}

error_state VehicleControl::communicate() {
	return error_state::SUCCESS;
}

error_state VehicleControl::processSharedMemory() {
	return error_state::SUCCESS;
}

// Stop when TMM sets this module's shutdown bit, or when TMM itself has stopped running.
bool VehicleControl::getShutdownFlag() {
	bool shutdownRequested = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdownRequested = (SM_TM_->shutdown & bit_VC) != 0;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdownRequested || tmmHasStopped();
}

void VehicleControl::threadFunction() {
	Console::WriteLine("VehicleControl: thread started (stub - Part 5 comes in the full build).");

	waitForStartBarrier();

	// Stub loop: nothing to do yet except tell TMM this thread is still alive.
	while (!getShutdownFlag()) {
		heartbeat();
		Thread::Sleep(STUB_LOOP_PERIOD_MS);
	}
	Console::WriteLine("VehicleControl: thread shut down.");
}

error_state VehicleControl::authenticate() {
	return error_state::SUCCESS;
}

String^ VehicleControl::buildCommand() {
	return "# 0 0 0 #";
}

// Tell TMM this thread is alive: TMM watches for this bit and clears it each time it sees it.
void VehicleControl::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_VC;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}
