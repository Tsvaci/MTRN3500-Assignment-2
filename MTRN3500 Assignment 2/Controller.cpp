#include "Controller.h"

// PROGRESS-CHECK BRANCH: this module runs as a heartbeat-only stub so that TMM has a real thread
// to start, monitor and shut down. Controller reading (Part 4) is implemented on the full-build branch.

Controller::Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC) {
	SM_TM_ = SM_TM;
	SM_VC_ = SM_VC;
	controller_ = nullptr;
	speed_ = 0.0;
	steering_ = 0.0;
}

Controller::~Controller() {
	delete controller_;
	controller_ = nullptr;
}

error_state Controller::processSharedMemory() {
	return error_state::SUCCESS;
}

bool Controller::getShutdownFlag() {
	bool flag = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		flag = (SM_TM_->shutdown & bit_CONTROLLER) != 0
			|| SM_TM_->WatchList[bitIndex(bit_TM)]->ElapsedMilliseconds > TM_ALIVE_TIMEOUT_MS;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return flag;
}

void Controller::threadFunction() {
	Console::WriteLine("Controller: thread started (stub - Part 4 is on the full-build branch).");

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
	Console::WriteLine("Controller: thread shut down.");
}

error_state Controller::readController() {
	return error_state::SUCCESS;
}

void Controller::requestShutdown() {}

void Controller::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_CONTROLLER;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}
