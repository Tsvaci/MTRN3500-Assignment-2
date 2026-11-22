#include "Controller.h"
#include "ConsoleView.h"

Controller::Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC) {
	SM_TM_ = SM_TM;
	SM_VC_ = SM_VC;
	controller_ = nullptr;
	speed_ = 0.0;
	steering_ = 0.0;
	wasConnected_ = true;
}

Controller::~Controller() {
	delete controller_;
	controller_ = nullptr;
}

// Speed: right trigger forward, left trigger reverse (keyboard mode: right stick up/down).
// Steering: right stick left/right. Stick right is +X, but steering is + to the left, so it is negated.
// Disconnected controller -> zero speed and steering.
error_state Controller::readController() {
	controllerState state = controller_->GetState();

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

double Controller::applyDeadzone(double value) {
	if (Math::Abs(value) < DEADZONE) {
		return 0.0;
	}
	return Math::Max(-1.0, Math::Min(1.0, value));
}

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

// Ask TMM to shut down; TMM then stops every thread, the same path as the 'q' key.
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

void Controller::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_CONTROLLER;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
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
	Console::WriteLine("Controller: thread started (" + (INPUT_TYPE == 1 ? "keyboard" : "Xbox controller") + " mode).");

	try {
		controller_ = new ControllerInterface(PLAYER_NUMBER, INPUT_TYPE);
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

	// Leave a stop command behind.
	speed_ = 0.0;
	steering_ = 0.0;
	processSharedMemory();
	delete controller_;
	controller_ = nullptr;
	Console::WriteLine("Controller: thread shut down.");
}
