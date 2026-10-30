#include "VC.h"

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
	return true;
}

void VehicleControl::threadFunction() {}

error_state VehicleControl::authenticate() {
	return error_state::SUCCESS;
}

String^ VehicleControl::buildCommand() {
	return "# 0 0 0 #";
}

void VehicleControl::heartbeat() {}
