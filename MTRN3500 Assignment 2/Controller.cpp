#include "Controller.h"

// Progress-check stub: heartbeat only.

static const int STUB_LOOP_PERIOD_MS = 100;

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
	bool shutdownRequested = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdownRequested = (SM_TM_->shutdown & bit_CONTROLLER) != 0;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdownRequested || tmmHasStopped();
}

void Controller::threadFunction() {
	Console::WriteLine("Controller: thread started (stub - Part 4 comes in the full build).");

	waitForStartBarrier();

	while (!getShutdownFlag()) {
		heartbeat();
		Thread::Sleep(STUB_LOOP_PERIOD_MS);
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
