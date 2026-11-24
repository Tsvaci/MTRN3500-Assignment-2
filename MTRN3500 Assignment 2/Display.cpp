#include "Display.h"

Display::Display(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser) {
	SM_TM_ = SM_TM;
	SM_Laser_ = SM_Laser;
	displayX_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	displayY_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	Client = nullptr;
	Stream = nullptr;
	ReadData = nullptr;
}

// runner.m has to be running in MATLAB before we start, or this fails.
error_state Display::connect(String^ hostName, int portNumber) {
	try {
		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;
		Client->SendTimeout = SEND_TIMEOUT_MS;
		Stream = Client->GetStream();
	}
	catch (Exception^ e) {
		Console::WriteLine("Display: could not connect to the MATLAB display at " + hostName + ":" + portNumber +
			" (" + e->Message + "). Is runner.m running?");
		return error_state::ERR_CONNECTION;
	}
	Console::WriteLine("Display: connected to " + hostName + ":" + portNumber);
	return error_state::SUCCESS;
}

// SM_Laser is in metres with x forward and y left. The MATLAB engine wants millimetres, with
// its X array going across the screen (+ right) and Y going forward. So display X = -y and
// display Y = x. Sending x and y as they are gives a plot turned 90 degrees.
error_state Display::processSharedMemory() {
	if (SM_Laser_ == nullptr) {
		return error_state::ERR_SM;
	}
	Monitor::Enter(SM_Laser_->lockObject);
	try {
		for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
			displayX_[i] = -SM_Laser_->y[i] * 1000.0;
			displayY_[i] = SM_Laser_->x[i] * 1000.0;
		}
	}
	finally {
		Monitor::Exit(SM_Laser_->lockObject);
	}
	return error_state::SUCCESS;
}

// The routine from the spec. Each array goes as raw bytes (361 doubles = 2888 bytes), X first.
void Display::sendDisplayData(array<double>^ xData, array<double>^ yData) {
	array<Byte>^ dataX = gcnew array<Byte>(xData->Length * sizeof(double));
	Buffer::BlockCopy(xData, 0, dataX, 0, dataX->Length);
	array<Byte>^ dataY = gcnew array<Byte>(yData->Length * sizeof(double));
	Buffer::BlockCopy(yData, 0, dataY, 0, dataY->Length);

	Stream->Write(dataX, 0, dataX->Length);
	Thread::Sleep(X_TO_Y_GAP_MS);
	Stream->Write(dataY, 0, dataY->Length);
}

// Send the latest scan to MATLAB.
error_state Display::communicate() {
	try {
		sendDisplayData(displayX_, displayY_);
	}
	catch (Exception^ e) {
		Console::WriteLine("Display: send failed - " + e->Message);
		return error_state::ERR_CONNECTION;
	}
	return error_state::SUCCESS;
}

// Heartbeat for TMM.
void Display::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_DISPLAY;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

// Stop if TMM says so, or if TMM itself has gone quiet.
bool Display::getShutdownFlag() {
	bool shutdownRequested = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdownRequested = (SM_TM_->shutdown & bit_DISPLAY) != 0;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdownRequested || tmmHasStopped();
}

// Connect, then send the scan every 100 ms. Critical.
void Display::threadFunction() {
	Console::WriteLine("Display: thread started.");

	error_state status = connect(DISPLAY_ADDRESS, DISPLAY_PORT);
	if (status != error_state::SUCCESS) {
		printError(status);
	}

	waitForStartBarrier();

	if (status != error_state::SUCCESS) {
		Console::WriteLine("Display: cannot run without a connection - thread stopping (TMM will notice the missing heartbeat).");
		return;
	}

	try {
		while (!getShutdownFlag()) {
			processSharedMemory();
			status = communicate();
			if (status != error_state::SUCCESS) {
				printError(status);
				break;
			}
			heartbeat();
			Thread::Sleep(SEND_PERIOD_MS);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("Display: unexpected error - " + e->Message);
	}

	try {
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
	}
	catch (Exception^) {}
	Console::WriteLine("Display: thread shut down.");
}
