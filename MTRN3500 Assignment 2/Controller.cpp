#include "Controller.h"

Controller::Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC) {
	SM_TM_ = SM_TM;
	SM_VC_ = SM_VC;
	controller_ = nullptr;
}

Controller::~Controller() {
	delete controller_;
	controller_ = nullptr;
}

error_state Controller::processSharedMemory() {
	return error_state::SUCCESS;
}

bool Controller::getShutdownFlag() {
	return true;
}

void Controller::threadFunction() {}

error_state Controller::readController() {
	return error_state::SUCCESS;
}

void Controller::requestShutdown() {}

void Controller::heartbeat() {}
