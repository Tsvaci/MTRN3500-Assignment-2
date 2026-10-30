#include "CrashAvoidance.h"

// PROGRESS-CHECK BRANCH: this module runs as a heartbeat-only stub so that TMM has a real thread
// to start, monitor and shut down. The decision logic (Part 7) is implemented on the full-build branch.

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
	bool flag = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		flag = (SM_TM_->shutdown & bit_CRASHAVOIDANCE) != 0
			|| SM_TM_->WatchList[bitIndex(bit_TM)]->ElapsedMilliseconds > TM_ALIVE_TIMEOUT_MS;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return flag;
}

void CrashAvoidance::threadFunction() {
	Console::WriteLine("CrashAvoidance: thread started (stub - Part 7 is on the full-build branch).");

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
