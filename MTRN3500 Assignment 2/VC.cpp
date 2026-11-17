#include "VC.h"

VehicleControl::VehicleControl(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC, SM_CrashAvoidance^ SM_CA) {
	SM_TM_ = SM_TM;
	SM_VC_ = SM_VC;
	SM_CA_ = SM_CA;
	requestedSpeed_ = 0.0;
	requestedSteering_ = 0.0;
	speed_ = 0.0;
	steering_ = 0.0;
	pathClearance_ = 0.0;
	crashAvoidanceStale_ = true;
	limited_ = false;
	watchdog_ = 0;
	statusTimer_ = Stopwatch::StartNew();
	Client = nullptr;
	Stream = nullptr;
	ReadData = nullptr;
}

error_state VehicleControl::connect(String^ hostName, int portNumber) {
	try {
		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;
		Client->ReceiveTimeout = RECEIVE_TIMEOUT_MS;
		Client->SendTimeout = SEND_TIMEOUT_MS;
		Stream = Client->GetStream();
		ReadData = gcnew array<unsigned char>(256);
	}
	catch (Exception^ e) {
		Console::WriteLine("VehicleControl: could not connect to " + hostName + ":" + portNumber + " (" + e->Message + ")");
		return error_state::ERR_CONNECTION;
	}
	Console::WriteLine("VehicleControl: connected to " + hostName + ":" + portNumber);
	return authenticate();
}

// Same as the LiDAR: send "<zID>\n", expect "OK".
error_state VehicleControl::authenticate() {
	try {
		array<unsigned char>^ request = Encoding::ASCII->GetBytes(STUDENT_ID "\n");
		Stream->Write(request, 0, request->Length);

		int bytesReceived = Stream->Read(ReadData, 0, ReadData->Length);
		if (bytesReceived <= 0) {
			return error_state::ERR_NO_DATA;
		}
		String^ reply = Encoding::ASCII->GetString(ReadData, 0, bytesReceived);
		if (!reply->StartsWith("OK")) {
			Console::WriteLine("VehicleControl: authentication rejected, server said: " + reply->Trim());
			return error_state::ERR_AUTHENTICATION;
		}
	}
	catch (IO::IOException^) {
		return error_state::ERR_TIMEOUT;
	}
	catch (Exception^ e) {
		Console::WriteLine("VehicleControl: authentication error - " + e->Message);
		return error_state::ERR_CONNECTION;
	}
	Console::WriteLine("VehicleControl: authenticated.");
	return error_state::SUCCESS;
}

// Reads the driver's command and the crash-avoidance limits, then works out what to send.
// Limits only apply to forward motion: there is no LiDAR coverage behind the vehicle.
// If crash avoidance has stopped updating (thread down or restarting), the vehicle is held still.
error_state VehicleControl::processSharedMemory() {
	if (SM_VC_ == nullptr || SM_CA_ == nullptr) {
		return error_state::ERR_SM;
	}

	Monitor::Enter(SM_VC_->lockObject);
	try {
		requestedSpeed_ = SM_VC_->Speed;
		requestedSteering_ = SM_VC_->Steering;
	}
	finally {
		Monitor::Exit(SM_VC_->lockObject);
	}

	double maxForwardSpeed;
	double maxLeftSteer;
	double maxRightSteer;
	bool stale;
	Monitor::Enter(SM_CA_->lockObject);
	try {
		maxForwardSpeed = SM_CA_->maxForwardSpeed;
		maxLeftSteer = SM_CA_->maxLeftSteer;
		maxRightSteer = SM_CA_->maxRightSteer;
		pathClearance_ = SM_CA_->pathClearance;
		stale = SM_CA_->lastUpdate->ElapsedMilliseconds > CA_STALE_MS;
	}
	finally {
		Monitor::Exit(SM_CA_->lockObject);
	}

	if (stale && !crashAvoidanceStale_) {
		Console::WriteLine("VehicleControl: crash avoidance not updating - holding the vehicle still.");
	}
	else if (!stale && crashAvoidanceStale_) {
		Console::WriteLine("VehicleControl: crash avoidance active.");
	}
	crashAvoidanceStale_ = stale;

	speed_ = Math::Max(-MAX_SPEED_MPS, Math::Min(MAX_SPEED_MPS, requestedSpeed_));
	steering_ = Math::Max(-MAX_STEER_DEG, Math::Min(MAX_STEER_DEG, requestedSteering_));

	double askedSpeed = speed_;
	double askedSteering = steering_;
	if (stale) {
		speed_ = 0.0;
	}
	else if (speed_ > 0.0) {
		speed_ = Math::Min(speed_, maxForwardSpeed);
		steering_ = Math::Max(-maxRightSteer, Math::Min(maxLeftSteer, steering_));
	}
	limited_ = (speed_ != askedSpeed || steering_ != askedSteering);
	return error_state::SUCCESS;
}

String^ VehicleControl::buildCommand() {
	return String::Format("# {0:F3} {1:F3} {2} #", steering_, speed_, watchdog_);
}

// The watchdog must change on every message or the vehicle stops.
error_state VehicleControl::communicate() {
	try {
		array<unsigned char>^ command = Encoding::ASCII->GetBytes(buildCommand());
		Stream->Write(command, 0, command->Length);
		watchdog_ = 1 - watchdog_;

		// Discard anything the server sends back so its buffer can't fill.
		while (Client->Available > 0) {
			Stream->Read(ReadData, 0, Math::Min(Client->Available, ReadData->Length));
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("VehicleControl: send failed - " + e->Message);
		return error_state::ERR_CONNECTION;
	}
	return error_state::SUCCESS;
}

// Twice a second while the driver is asking for motion, show what crash avoidance changed.
void VehicleControl::printStatus() {
	if (statusTimer_->ElapsedMilliseconds < STATUS_PERIOD_MS) {
		return;
	}
	if (requestedSpeed_ == 0.0 && requestedSteering_ == 0.0) {
		return;
	}
	statusTimer_->Restart();

	String^ note = "";
	if (crashAvoidanceStale_) {
		note = "  HELD (crash avoidance not running)";
	}
	else if (limited_) {
		note = String::Format("  LIMITED (clear path {0:F2} m)", pathClearance_);
	}
	Console::WriteLine(String::Format("VehicleControl: asked speed {0,5:F2} steer {1,6:F1} -> sent speed {2,5:F2} steer {3,6:F1}{4}",
		requestedSpeed_, requestedSteering_, speed_, steering_, note));
}

void VehicleControl::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_VC;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

bool VehicleControl::getShutdownFlag() {
	bool shutdownRequested = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdownRequested = (SM_TM_->shutdown & bit_VC) != 0;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdownRequested || tmmHasStopped();
}

void VehicleControl::threadFunction() {
	Console::WriteLine("VehicleControl: thread started.");

	error_state status = connect(WEEDER_ADDRESS, VC_PORT);
	if (status != error_state::SUCCESS) {
		printError(status);
	}

	waitForStartBarrier();

	if (status != error_state::SUCCESS) {
		Console::WriteLine("VehicleControl: cannot run without a connection - thread stopping (TMM will notice the missing heartbeat).");
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
			printStatus();
			Thread::Sleep(SEND_PERIOD_MS);
		}

		// Final stop command before closing.
		speed_ = 0.0;
		steering_ = 0.0;
		if (status == error_state::SUCCESS) {
			communicate();
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("VehicleControl: unexpected error - " + e->Message);
	}

	try {
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
	}
	catch (Exception^) {}
	Console::WriteLine("VehicleControl: thread shut down.");
}
