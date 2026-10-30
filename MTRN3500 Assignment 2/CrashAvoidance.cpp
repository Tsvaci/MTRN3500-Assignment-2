#include "CrashAvoidance.h"

CrashAvoidance::CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser, SM_VehicleControl^ SM_VC, SM_CrashAvoidance^ SM_CA) {
	SM_TM_ = SM_TM;
	SM_Laser_ = SM_Laser;
	SM_VC_ = SM_VC;
	SM_CA_ = SM_CA;
	x_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	y_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	speed_ = 0.0;
	steering_ = 0.0;
	inhibit_ = false;
	nearestObstacle_ = 0.0;
}

error_state CrashAvoidance::processSharedMemory() {
	return error_state::SUCCESS;
}

bool CrashAvoidance::getShutdownFlag() {
	return true;
}

void CrashAvoidance::threadFunction() {}

bool CrashAvoidance::collisionLikely() {
	return false;
}

void CrashAvoidance::heartbeat() {}
