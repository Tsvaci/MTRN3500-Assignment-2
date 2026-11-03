#include "CrashAvoidance.h"

// Progress-check stub: heartbeat only.

static const int STUB_LOOP_PERIOD_MS = 100;

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
	bool shutdownRequested = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdownRequested = (SM_TM_->shutdown & bit_CRASHAVOIDANCE) != 0;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdownRequested || tmmHasStopped();
}

void CrashAvoidance::threadFunction() {
	Console::WriteLine("CrashAvoidance: thread started (stub - Part 7 comes in the full build).");

	waitForStartBarrier();

	while (!getShutdownFlag()) {
		heartbeat();
		Thread::Sleep(STUB_LOOP_PERIOD_MS);
	}
	Console::WriteLine("CrashAvoidance: thread shut down.");
}

bool CrashAvoidance::collisionLikely() {
	return false;
}

void CrashAvoidance::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_CRASHAVOIDANCE;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}
