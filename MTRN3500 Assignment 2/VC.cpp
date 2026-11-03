#include "VC.h"

// Progress-check stub: heartbeat only.

static const int STUB_LOOP_PERIOD_MS = 100;

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

void VehicleControl::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_VC;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}
