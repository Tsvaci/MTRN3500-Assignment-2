#include "LiDAR.h"
#include "ConsoleView.h"

LiDAR::LiDAR(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser) {
	SM_TM_ = SM_TM;
	SM_Laser_ = SM_Laser;
	scanX_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	scanY_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	scanCount_ = 0;
	scansAtLastPrint_ = 0;
	printTimer_ = Stopwatch::StartNew();
	Client = nullptr;
	Stream = nullptr;
	ReadData = nullptr;
}

// Connect with 1 s timeouts so a dead robot can't hang the thread, then log in.
error_state LiDAR::connect(String^ hostName, int portNumber) {
	try {
		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;
		Client->ReceiveTimeout = RECEIVE_TIMEOUT_MS;
		Client->SendTimeout = SEND_TIMEOUT_MS;
		Stream = Client->GetStream();
		ReadData = gcnew array<unsigned char>(READ_BUFFER_SIZE);
	}
	catch (SocketException^ e) {
		Console::WriteLine("LiDAR: could not connect to " + hostName + ":" + portNumber + " (" + e->Message + ")");
		return error_state::ERR_CONNECTION;
	}
	catch (Exception^ e) {
		Console::WriteLine("LiDAR: connection error - " + e->Message);
		return error_state::ERR_CONNECTION;
	}
	Console::WriteLine("LiDAR: connected to " + hostName + ":" + portNumber);
	return authenticate();
}

// Send our zID and expect "OK" back.
error_state LiDAR::authenticate() {
	try {
		array<unsigned char>^ request = Encoding::ASCII->GetBytes(STUDENT_ID "\n");
		Stream->Write(request, 0, request->Length);

		int bytesReceived = Stream->Read(ReadData, 0, ReadData->Length);
		if (bytesReceived <= 0) {
			return error_state::ERR_NO_DATA;
		}
		String^ reply = Encoding::ASCII->GetString(ReadData, 0, bytesReceived);
		if (!reply->StartsWith("OK")) {
			Console::WriteLine("LiDAR: authentication rejected, server said: " + reply->Trim());
			return error_state::ERR_AUTHENTICATION;
		}
	}
	catch (IO::IOException^) {
		return error_state::ERR_TIMEOUT;
	}
	catch (Exception^ e) {
		Console::WriteLine("LiDAR: authentication error - " + e->Message);
		return error_state::ERR_CONNECTION;
	}
	Console::WriteLine("LiDAR: authenticated.");
	return error_state::SUCCESS;
}

// Ask for one scan. The command has to be wrapped in STX and ETX.
error_state LiDAR::requestScan() {
	try {
		array<unsigned char>^ request = Encoding::ASCII->GetBytes("sRN LMDscandata");
		array<unsigned char>^ framedRequest = gcnew array<unsigned char>(request->Length + 2);
		framedRequest[0] = STX;
		Array::Copy(request, 0, framedRequest, 1, request->Length);
		framedRequest[framedRequest->Length - 1] = ETX;
		Stream->Write(framedRequest, 0, framedRequest->Length);
	}
	catch (Exception^ e) {
		Console::WriteLine("LiDAR: failed to send scan request - " + e->Message);
		return error_state::ERR_CONNECTION;
	}
	return error_state::SUCCESS;
}

// TCP doesn't keep a reply in one piece, so keep reading until the ETX byte turns up.
error_state LiDAR::readTelegram(String^% telegram) {
	int totalBytes = 0;
	try {
		while (true) {
			if (totalBytes >= ReadData->Length) {
				Console::WriteLine("LiDAR: telegram larger than the read buffer - discarded.");
				return error_state::ERR_INVALID_DATA;
			}
			int bytesReceived = Stream->Read(ReadData, totalBytes, ReadData->Length - totalBytes);
			if (bytesReceived <= 0) {
				return error_state::ERR_CONNECTION;			// Server closed the connection
			}
			totalBytes += bytesReceived;
			if (Array::IndexOf(ReadData, ETX, totalBytes - bytesReceived, bytesReceived) >= 0) {
				break;
			}
		}
	}
	catch (IO::IOException^) {
		return error_state::ERR_TIMEOUT;
	}
	catch (Exception^ e) {
		Console::WriteLine("LiDAR: read error - " + e->Message);
		return error_state::ERR_CONNECTION;
	}

	int stxIndex = Array::IndexOf(ReadData, STX, 0, totalBytes);
	int etxIndex = Array::IndexOf(ReadData, ETX, 0, totalBytes);
	if (stxIndex < 0 || etxIndex < 0 || etxIndex <= stxIndex) {
		return error_state::ERR_INVALID_DATA;
	}
	telegram = Encoding::ASCII->GetString(ReadData, stxIndex + 1, etxIndex - stxIndex - 1);
	return error_state::SUCCESS;
}

// The fields after DIST1 are hex: scale factor, scale offset, start angle, angle step, point count,
// then one range per point in mm. Angles run 0 = right, 90 = straight ahead, 180 = left.
// We store x forward, y left, in metres. A range of 0 means no echo and ends up at (0, 0).
// The real sensor answers "sRA" but the simulator answers "sRN", so either is accepted.
error_state LiDAR::parseScan(String^ telegram) {
	array<String^>^ fields = telegram->Split(' ');

	if (fields->Length < 3 || (fields[0] != "sRA" && fields[0] != "sRN") || fields[1] != "LMDscandata") {
		Console::WriteLine("LiDAR: unexpected reply: " + (telegram->Length > 40 ? telegram->Substring(0, 40) : telegram));
		return error_state::ERR_INVALID_DATA;
	}

	int dist1Index = Array::IndexOf(fields, "DIST1");
	int firstDistanceIndex = dist1Index + 6;
	if (dist1Index < 0 || firstDistanceIndex > fields->Length) {
		return error_state::ERR_INVALID_DATA;
	}

	try {
		// Scale factor is an IEEE 754 float sent as hex.
		unsigned int scaleFactorBits = Convert::ToUInt32(fields[dist1Index + 1], 16);
		float scaleFactor = BitConverter::ToSingle(BitConverter::GetBytes(scaleFactorBits), 0);
		// Scale offset (dist1Index + 2) is always 0.
		int startAngle = Convert::ToInt32(fields[dist1Index + 3], 16);			// 1/10000 deg
		unsigned int angleStep = Convert::ToUInt16(fields[dist1Index + 4], 16);	// 1/10000 deg
		int pointCount = Convert::ToUInt16(fields[dist1Index + 5], 16);

		if (pointCount != STANDARD_LASER_LENGTH || firstDistanceIndex + pointCount > fields->Length) {
			Console::WriteLine("LiDAR: expected " + STANDARD_LASER_LENGTH + " points, telegram has " + pointCount);
			return error_state::ERR_INVALID_DATA;
		}
		if (scaleFactor <= 0.0f || angleStep == 0) {
			return error_state::ERR_INVALID_DATA;
		}
		// gives the distance and angle of each point, and turns it into an x and y coordinate
		for (int i = 0; i < pointCount; i++) {
			unsigned int distanceMm = Convert::ToUInt16(fields[firstDistanceIndex + i], 16);
			double rangeM = distanceMm * scaleFactor / 1000.0;
			double angleDeg = (startAngle + (double)i * angleStep) / 10000.0;
			double angleRad = angleDeg * Math::PI / 180.0;
			scanX_[i] = rangeM * Math::Sin(angleRad);
			scanY_[i] = -rangeM * Math::Cos(angleRad);
		}
	}
	catch (FormatException^) {
		return error_state::ERR_INVALID_DATA;
	}
	catch (OverflowException^) {
		return error_state::ERR_INVALID_DATA;
	}
	return error_state::SUCCESS;
}

// One scan: request it, read the reply, decode it.
error_state LiDAR::communicate() {
	error_state result = requestScan();
	if (result != error_state::SUCCESS) {
		return result;
	}
	String^ telegram = nullptr;
	result = readTelegram(telegram);
	if (result != error_state::SUCCESS) {
		return result;
	}
	return parseScan(telegram);
}

// Copy the scan into shared memory and bump the counter so readers can tell it's a new one.
error_state LiDAR::processSharedMemory() {
	if (SM_Laser_ == nullptr) {
		return error_state::ERR_SM;
	}
	Monitor::Enter(SM_Laser_->lockObject);
	try {
		Array::Copy(scanX_, SM_Laser_->x, STANDARD_LASER_LENGTH);
		Array::Copy(scanY_, SM_Laser_->y, STANDARD_LASER_LENGTH);
		SM_Laser_->scanNumber++;
	}
	finally {
		Monitor::Exit(SM_Laser_->lockObject);
	}
	return error_state::SUCCESS;
}

// Every 2 s, print the scan rate and the nearest point (direction measured from straight ahead).
// It's built as one string first so another thread can't print in the middle of it.
void LiDAR::printScan() {
	bool view = ConsoleView::isActive();
	if (printTimer_->ElapsedMilliseconds < (view ? VIEW_PERIOD_MS : PRINT_PERIOD_MS)) {
		return;
	}
	double seconds = printTimer_->ElapsedMilliseconds / 1000.0;
	double scansPerSecond = (scanCount_ - scansAtLastPrint_) / seconds;
	printTimer_->Restart();
	scansAtLastPrint_ = scanCount_;

	if (view || !PRINT_ALL_POINTS) {
		int nearest = -1;
		double nearestRange = 0.0;
		for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
			double range = Math::Sqrt(scanX_[i] * scanX_[i] + scanY_[i] * scanY_[i]);
			if (range >= MIN_VALID_RANGE_M && (nearest < 0 || range < nearestRange)) {
				nearest = i;
				nearestRange = range;
			}
		}
		String^ nearestText = "nothing in range";
		ConsoleColor colour = ConsoleColor::White;
		if (nearest >= 0) {
			double angle = Math::Atan2(scanY_[nearest], scanX_[nearest]) * 180.0 / Math::PI;   // + left of ahead
			String^ side = Math::Abs(angle) < 2.5 ? "ahead" : String::Format("{0:F0} deg {1}", Math::Abs(angle), angle > 0 ? "left" : "right");
			nearestText = String::Format("nearest {0:F2} m, {1}", nearestRange, side);
			if (nearestRange < 1.0) {
				colour = ConsoleColor::DarkYellow;              // Within the 1 m crash-avoidance zone
			}
		}
		if (view) {
			ConsoleView::set("lidar", String::Format("{0:F0} scans/s    {1}", scansPerSecond, nearestText), colour);
			return;
		}
		Console::WriteLine(String::Format("LiDAR: {0,4:F0} scans/s   {1}", scansPerSecond, nearestText));
		return;
	}

	Text::StringBuilder^ text = gcnew Text::StringBuilder();
	text->AppendLine("LiDAR scan #" + scanCount_ + "  (" + STANDARD_LASER_LENGTH + " points, metres, x = forward, y = left)");
	for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
		text->Append(String::Format("{0,3}: ({1,6:F2}, {2,6:F2})   ", i, scanX_[i], scanY_[i]));
		if (i % 4 == 3 || i == STANDARD_LASER_LENGTH - 1) {
			text->AppendLine();
		}
	}
	Console::Write(text->ToString());
}

// Heartbeat for TMM.
void LiDAR::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_LASER;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

// Stop if TMM says so, or if TMM itself has gone quiet.
bool LiDAR::getShutdownFlag() {
	bool shutdownRequested = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdownRequested = (SM_TM_->shutdown & bit_LASER) != 0;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdownRequested || tmmHasStopped();
}

// Connect, then ask for a scan every 20 ms. Critical: if this thread ends, TMM shuts everything down.
void LiDAR::threadFunction() {
	Console::WriteLine("LiDAR: thread started.");

	error_state status = connect(WEEDER_ADDRESS, LIDAR_PORT);
	if (status != error_state::SUCCESS) {
		printError(status);
	}

	// Signal the barrier even on failure so the other threads aren't held up.
	waitForStartBarrier();

	if (status != error_state::SUCCESS) {
		Console::WriteLine("LiDAR: cannot run without a connection - thread stopping (TMM will notice the missing heartbeat).");
		return;
	}

	int consecutiveTimeouts = 0;
	try {
		while (!getShutdownFlag()) {
			status = communicate();
			heartbeat();

			if (status == error_state::SUCCESS) {
				consecutiveTimeouts = 0;
				processSharedMemory();
				scanCount_++;
				printScan();
			}
			else {
				printError(status);
				if (status == error_state::ERR_CONNECTION) {
					break;
				}
				// Still connected but nothing's coming back. Give up so TMM notices the missing heartbeat.
				if (status == error_state::ERR_TIMEOUT && ++consecutiveTimeouts >= MAX_CONSECUTIVE_TIMEOUTS) {
					Console::WriteLine("LiDAR: no data for " + consecutiveTimeouts + " requests in a row - giving up.");
					break;
				}
			}
			Thread::Sleep(SCAN_PERIOD_MS);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("LiDAR: unexpected error - " + e->Message);
	}

	try {
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
	}
	catch (Exception^) {}
	Console::WriteLine("LiDAR: thread shut down after " + scanCount_ + " scans.");
}
