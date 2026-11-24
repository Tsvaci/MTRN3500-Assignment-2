#include "Controller.h"
#include "ConsoleView.h"

Controller::Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC) {
	SM_TM_ = SM_TM;
	SM_VC_ = SM_VC;
	controllerInterface_ = nullptr;
	speed_ = 0.0;
	steering_ = 0.0;
	wasConnected_ = true;
}

Controller::~Controller() {
	delete controllerInterface_;
	controllerInterface_ = nullptr;
}

// Speed is right trigger minus left trigger (in keyboard mode it's the right stick up/down).
// The stick reads + for right, but our steering is + for left, hence the minus sign.
// If the controller is unplugged, speed and steering go to zero.
error_state Controller::readController() {
	controllerState state = controllerInterface_->GetState();

	if (!state.isConnected) {
		speed_ = 0.0;
		steering_ = 0.0;
		ConsoleView::set("controller", "DISCONNECTED - speed and steering held at zero", ConsoleColor::Red);
		if (wasConnected_) {
			Console::WriteLine("Controller: DISCONNECTED - speed and steering set to zero.");
		}
		wasConnected_ = false;
		return error_state::ERR_NO_DATA;
	}
	if (!wasConnected_) {
		Console::WriteLine("Controller: reconnected.");
	}
	wasConnected_ = true;
	ConsoleView::set("controller", "connected", ConsoleColor::Green);

	double throttle;
	if (INPUT_TYPE == 1) {
		throttle = state.rightThumbY;                        // Keyboard mode has no triggers
	}
	else {
		throttle = state.rightTrigger - state.leftTrigger;
	}
	speed_ = applyDeadzone(throttle) * MAX_SPEED_MPS;
	steering_ = -applyDeadzone(state.rightThumbX) * MAX_STEER_DEG;

	if (state.buttonB) {
		requestShutdown();
	}
	return error_state::SUCCESS;
}

// Sticks never sit at exactly zero, so ignore small readings or the robot would creep.
double Controller::applyDeadzone(double value) {
	if (Math::Abs(value) < DEADZONE) {
		return 0.0;
	}
	return Math::Max(-1.0, Math::Min(1.0, value));
}

// Write speed and steering to SM_VehicleControl.
error_state Controller::processSharedMemory() {
	if (SM_VC_ == nullptr) {
		return error_state::ERR_SM;
	}
	Monitor::Enter(SM_VC_->lockObject);
	try {
		SM_VC_->Speed = speed_;
		SM_VC_->Steering = steering_;
	}
	finally {
		Monitor::Exit(SM_VC_->lockObject);
	}
	return error_state::SUCCESS;
}

// B button. We only set TMM's flag and TMM does the actual shutdown, same as pressing q.
void Controller::requestShutdown() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		if ((SM_TM_->shutdown & bit_TM) == 0) {
			Console::WriteLine("Controller: B pressed - shutdown requested.");
			SM_TM_->shutdown |= bit_TM;
		}
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

// Heartbeat for TMM.
void Controller::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_CONTROLLER;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

// Stop if TMM says so, or if TMM itself has gone quiet.
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

// Read the controller every 20 ms. Critical.
void Controller::threadFunction() {
	Console::WriteLine("Controller: thread started (" + (INPUT_TYPE == 1 ? "keyboard" : "Xbox controller") + " mode).");

	try {
		controllerInterface_ = new ControllerInterface(PLAYER_NUMBER, INPUT_TYPE);
	}
	catch (...) {
		Console::WriteLine("Controller: could not create the controller interface - thread stopping.");
		waitForStartBarrier();
		return;
	}

	waitForStartBarrier();

	try {
		while (!getShutdownFlag()) {
			readController();             // On failure speed_ and steering_ are already zero
			processSharedMemory();
			heartbeat();
			Thread::Sleep(POLL_PERIOD_MS);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("Controller: unexpected error - " + e->Message);
	}

	// Leave zero speed on the board on the way out.
	speed_ = 0.0;
	steering_ = 0.0;
	processSharedMemory();
	delete controllerInterface_;
	controllerInterface_ = nullptr;
	Console::WriteLine("Controller: thread shut down.");
}
