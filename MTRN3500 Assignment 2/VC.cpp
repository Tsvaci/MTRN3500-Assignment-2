#include "VC.h"

// PROGRESS-CHECK BRANCH: this module runs as a heartbeat-only stub so that TMM has a real thread
// to start, monitor and shut down. Vehicle control (Part 5) is implemented on the full-build branch.

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
	bool flag = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		flag = (SM_TM_->shutdown & bit_VC) != 0
			|| SM_TM_->WatchList[bitIndex(bit_TM)]->ElapsedMilliseconds > TM_ALIVE_TIMEOUT_MS;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return flag;
}

void VehicleControl::threadFunction() {
	Console::WriteLine("VehicleControl: thread started (stub - Part 5 is on the full-build branch).");

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
