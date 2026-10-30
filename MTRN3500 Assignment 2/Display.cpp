#include "Display.h"

Display::Display(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser) {
	SM_TM_ = SM_TM;
	SM_Laser_ = SM_Laser;
	x_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	y_ = gcnew array<double>(STANDARD_LASER_LENGTH);
}

error_state Display::connect(String^ hostName, int portNumber) {
	return error_state::SUCCESS;
}

error_state Display::communicate() {
	return error_state::SUCCESS;
}

error_state Display::processSharedMemory() {
	return error_state::SUCCESS;
}

bool Display::getShutdownFlag() {
	return true;
}

void Display::threadFunction() {}

void Display::sendDisplayData(array<double>^ xData, array<double>^ yData) {}

void Display::heartbeat() {}
