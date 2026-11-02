#include "TMM.h"
#include <conio.h>		// Lets us check the keyboard for 'q'

ThreadManagement::ThreadManagement() {
	SM_TM_ = nullptr;
	SM_Laser_ = nullptr;
	SM_GPS_ = nullptr;
	SM_VC_ = nullptr;
	SM_CA_ = nullptr;
}

// Make all the shared memory. TMM owns it and gives each module the parts it needs.
error_state ThreadManagement::setupSharedMemory() {
	try {
		SM_TM_ = gcnew SM_ThreadManagement();
		SM_TM_->ThreadBarrier = gcnew Barrier(THREAD_COUNT);
		SM_TM_->WatchList = gcnew array<Stopwatch^>(THREAD_COUNT);
		for (int i = 0; i < THREAD_COUNT; i++) {
			SM_TM_->WatchList[i] = gcnew Stopwatch();
		}
		SM_TM_->WatchList[bitIndex(bit_TM)]->Start();	// TMM counts as running from now

		SM_Laser_ = gcnew SM_Laser();
		SM_GPS_ = gcnew SM_GPS();
		SM_VC_ = gcnew SM_VehicleControl();
		SM_CA_ = gcnew SM_CrashAvoidance();
	}
	catch (Exception^ e) {
		Console::WriteLine("TMM: shared memory setup failed: " + e->Message);
		return error_state::ERR_SM;
	}
	return error_state::SUCCESS;
}

// Tell the other modules that TMM is still running.
error_state ThreadManagement::processSharedMemory() {
	if (SM_TM_ == nullptr) {
		return error_state::ERR_SM;
	}
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_TM;
		SM_TM_->WatchList[bitIndex(bit_TM)]->Restart();
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return error_state::SUCCESS;
}

// Start all six module threads, in the order listed in TMM.h.
error_state ThreadManagement::startThreads() {
	for each (uint8_t moduleBit in MODULE_START_ORDER) {
		error_state result = startModule(moduleBit);
		if (result != error_state::SUCCESS) {
			printError(result);
			return result;
		}
	}

	// Wait until every thread has finished getting ready, so they all start together.
	// Give up waiting after a while, so one stuck thread can't hold everything up.
	Console::WriteLine("TMM: waiting for all threads to finish starting...");
	bool allStarted = SM_TM_->ThreadBarrier->SignalAndWait(BARRIER_TIMEOUT_MS);
	if (!allStarted) {
		Console::WriteLine("TMM: WARNING - not all threads reached the start barrier in time.");
	}

	// Start-up is over. Remove the barrier so a thread that gets restarted later doesn't wait on it.
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->ThreadBarrier = nullptr;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}

	Console::WriteLine("TMM: all threads running. Press 'q' to shut down.");
	return error_state::SUCCESS;
}

// Create one module and start its thread. Also used to restart a module.
error_state ThreadManagement::startModule(uint8_t moduleBit) {
	try {
		Thread^ thread = nullptr;

		switch (moduleBit) {
		case bit_CONTROLLER:
			controller_ = gcnew Controller(SM_TM_, SM_VC_);
			thread = gcnew Thread(gcnew ThreadStart(controller_, &Controller::threadFunction));
			controllerThread_ = thread;
			break;
		case bit_LASER:
			lidar_ = gcnew LiDAR(SM_TM_, SM_Laser_);
			thread = gcnew Thread(gcnew ThreadStart(lidar_, &LiDAR::threadFunction));
			lidarThread_ = thread;
			break;
		case bit_CRASHAVOIDANCE:
			crashAvoidance_ = gcnew CrashAvoidance(SM_TM_, SM_Laser_, SM_VC_, SM_CA_);
			thread = gcnew Thread(gcnew ThreadStart(crashAvoidance_, &CrashAvoidance::threadFunction));
			crashAvoidanceThread_ = thread;
			break;
		case bit_VC:
			vehicleControl_ = gcnew VehicleControl(SM_TM_, SM_VC_, SM_CA_);
			thread = gcnew Thread(gcnew ThreadStart(vehicleControl_, &VehicleControl::threadFunction));
			vehicleControlThread_ = thread;
			break;
		case bit_DISPLAY:
			display_ = gcnew Display(SM_TM_, SM_Laser_);
			thread = gcnew Thread(gcnew ThreadStart(display_, &Display::threadFunction));
			displayThread_ = thread;
			break;
		case bit_GPS:
			gnss_ = gcnew GNSS(SM_TM_, SM_GPS_);
			thread = gcnew Thread(gcnew ThreadStart(gnss_, &GNSS::threadFunction));
			gnssThread_ = thread;
			break;
		default:
			return error_state::ERR_STARTUP;
		}

		// Clear anything left over from before, and start this module's timer fresh.
		Monitor::Enter(SM_TM_->lockObject);
		try {
			SM_TM_->shutdown &= ~moduleBit;
			SM_TM_->heartbeat &= ~moduleBit;
			SM_TM_->WatchList[bitIndex(moduleBit)]->Restart();
		}
		finally {
			Monitor::Exit(SM_TM_->lockObject);
		}

		thread->Start();
	}
	catch (Exception^ e) {
		Console::WriteLine("TMM: failed to start " + moduleName(moduleBit) + ": " + e->Message);
		return error_state::ERR_STARTUP;
	}
	return error_state::SUCCESS;
}

// Check that every module is still running.
// Each module keeps ticking its bit. Each time TMM sees the tick, it resets that module's timer.
// If a timer runs too long, that module has stopped.
// An important (critical) module stopping means shut everything down.
// A less important one just gets restarted.
uint8_t ThreadManagement::checkHeartbeats() {
	// First, note which modules have ticked since last time.
	Monitor::Enter(SM_TM_->lockObject);
	try {
		for each (uint8_t moduleBit in MODULE_START_ORDER) {
			if (SM_TM_->heartbeat & moduleBit) {
				SM_TM_->heartbeat &= ~moduleBit;
				SM_TM_->WatchList[bitIndex(moduleBit)]->Restart();
			}
		}
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}

	// Then deal with any module that has gone quiet for too long.
	for each (uint8_t moduleBit in MODULE_START_ORDER) {
		if (SM_TM_->WatchList[bitIndex(moduleBit)]->ElapsedMilliseconds > HEARTBEAT_TIMEOUT_MS) {
			if (moduleBit & CRITICALMASK) {
				return moduleBit;
			}
			Console::WriteLine("TMM: " + moduleName(moduleBit) + " (non-critical) stopped responding - restarting it.");
			if (startModule(moduleBit) != error_state::SUCCESS) {
				Console::WriteLine("TMM: restart failed; will try again after the next timeout.");
			}
		}
	}
	return 0;
}

// True if 'q' was pressed in this window.
bool ThreadManagement::shutdownKeyPressed() {
	if (_kbhit()) {
		int key = _getch();
		return (key == 'q' || key == 'Q');
	}
	return false;
}

// Tell every module to stop, then wait for each one to finish.
void ThreadManagement::shutdownModules() {
	Console::WriteLine("TMM: shutting down all modules...");

	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->shutdown = bit_ALL;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}

	// Same order as MODULE_START_ORDER, so each thread matches its name.
	array<Thread^>^ threads = { controllerThread_, lidarThread_, crashAvoidanceThread_, vehicleControlThread_, displayThread_, gnssThread_ };

	for (int i = 0; i < threads->Length; i++) {
		if (threads[i] == nullptr) {
			continue;									// This one was never started
		}
		String^ name = moduleName(MODULE_START_ORDER[i]);
		if (threads[i]->Join(THREAD_STOP_WAIT_MS)) {
			Console::WriteLine("TMM: " + name + " thread stopped.");
		}
		else {
			Console::WriteLine("TMM: WARNING - " + name + " thread did not stop within " + THREAD_STOP_WAIT_MS + " ms.");
		}
	}
	Console::WriteLine("TMM: shutdown complete.");
}

// True if another module (such as the controller's stop button) has asked TMM to stop.
bool ThreadManagement::getShutdownFlag() {
	bool shutdownRequested = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdownRequested = (SM_TM_->shutdown & bit_TM) != 0;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdownRequested;
}

String^ ThreadManagement::moduleName(uint8_t moduleBit) {
	switch (moduleBit) {
	case bit_TM:             return "TMM";
	case bit_LASER:          return "LiDAR";
	case bit_GPS:            return "GNSS";
	case bit_VC:             return "VehicleControl";
	case bit_CONTROLLER:     return "Controller";
	case bit_DISPLAY:        return "Display";
	case bit_CRASHAVOIDANCE: return "CrashAvoidance";
	default:                 return "module " + moduleBit;
	}
}

// TMM's main job: start everything, keep checking on it, and shut it all down at the end.
void ThreadManagement::threadFunction() {
	Console::WriteLine("TMM: thread started.");

	if (startThreads() != error_state::SUCCESS) {
		Console::WriteLine("TMM: could not start all modules - shutting down.");
		shutdownModules();
		return;
	}

	try {
		while (true) {
			processSharedMemory();				// Tell the modules TMM is still running

			uint8_t deadCriticalModule = checkHeartbeats();
			if (deadCriticalModule != 0) {
				Console::WriteLine("TMM: CRITICAL module " + moduleName(deadCriticalModule) + " stopped responding.");
				break;
			}
			if (shutdownKeyPressed()) {
				Console::WriteLine("TMM: 'q' pressed - routine shutdown requested.");
				break;
			}
			if (getShutdownFlag()) {
				Console::WriteLine("TMM: shutdown requested by another module.");
				break;
			}
			Thread::Sleep(MONITOR_PERIOD_MS);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("TMM: unexpected error - " + e->Message);
	}

	shutdownModules();
	Console::WriteLine("TMM: thread finished.");
}
