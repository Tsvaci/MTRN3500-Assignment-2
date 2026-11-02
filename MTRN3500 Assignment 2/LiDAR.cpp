#include "LiDAR.h"

LiDAR::LiDAR(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser) {
	SM_TM_ = SM_TM;
	SM_Laser_ = SM_Laser;
	x_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	y_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	scanCount_ = 0;
	Client = nullptr;
	Stream = nullptr;
	ReadData = nullptr;
}

// Connect to the LiDAR and log in with the student ID.
error_state LiDAR::connect(String^ hostName, int portNumber) {
	try {
		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;						// Send each small message straight away
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

// Log in: send the student ID and a newline. The LiDAR answers "OK".
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

// Ask the LiDAR for its latest scan.
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

// Read the LiDAR's reply. It can arrive in several pieces, so keep reading until the end
// marker turns up. Then return the text between the start and end markers.
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
				return error_state::ERR_CONNECTION;			// The LiDAR closed the connection
			}
			totalBytes += bytesReceived;
			if (Array::IndexOf(ReadData, ETX, totalBytes - bytesReceived, bytesReceived) >= 0) {
				break;										// Found the end marker
			}
		}
	}
	catch (IO::IOException^) {
		return error_state::ERR_TIMEOUT;						// Waited too long for a reply
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

// Turn the LiDAR's reply into 361 x,y points.
// The reply is a line of words separated by spaces. Numbers are written in hex.
// After the word DIST1 come: scale factor, scale offset, start angle, angle step,
// number of points, then one distance per point in millimetres.
// Angles: 0 is the vehicle's right, 90 is straight ahead, 180 is its left.
// Output in metres: x is distance forward, y is distance to the left.
// The real LiDAR's reply starts with "sRA". The simulator's starts with "sRN". Both are accepted.
// A distance of 0 means nothing was detected. That point is stored as (0, 0).
error_state LiDAR::parseScan(String^ telegram) {
	array<String^>^ fields = telegram->Split(' ');

	if (fields->Length < 3 || (fields[0] != "sRA" && fields[0] != "sRN") || fields[1] != "LMDscandata") {
		Console::WriteLine("LiDAR: unexpected reply: " + (telegram->Length > 40 ? telegram->Substring(0, 40) : telegram));
		return error_state::ERR_INVALID_DATA;
	}

	// Everything we need comes after the word DIST1.
	int dist1Index = Array::IndexOf(fields, "DIST1");
	int firstDistanceIndex = dist1Index + 6;
	if (dist1Index < 0 || firstDistanceIndex > fields->Length) {
		return error_state::ERR_INVALID_DATA;
	}

	try {
		// The scale factor is a decimal number, sent in a coded hex form. This decodes it.
		unsigned int scaleFactorBits = Convert::ToUInt32(fields[dist1Index + 1], 16);
		float scaleFactor = BitConverter::ToSingle(BitConverter::GetBytes(scaleFactorBits), 0);
		// (The scale offset comes next. It is always 0, so it is skipped.)
		int startAngle = Convert::ToInt32(fields[dist1Index + 3], 16);			// In 1/10000ths of a degree
		unsigned int angleStep = Convert::ToUInt16(fields[dist1Index + 4], 16);	// In 1/10000ths of a degree
		int pointCount = Convert::ToUInt16(fields[dist1Index + 5], 16);

		if (pointCount != STANDARD_LASER_LENGTH || firstDistanceIndex + pointCount > fields->Length) {
			Console::WriteLine("LiDAR: expected " + STANDARD_LASER_LENGTH + " points, telegram has " + pointCount);
			return error_state::ERR_INVALID_DATA;
		}
		if (scaleFactor <= 0.0f || angleStep == 0) {
			return error_state::ERR_INVALID_DATA;
		}

		for (int i = 0; i < pointCount; i++) {
			unsigned int distanceMm = Convert::ToUInt16(fields[firstDistanceIndex + i], 16);
			double rangeM = distanceMm * scaleFactor / 1000.0;
			double angleDeg = (startAngle + (double)i * angleStep) / 10000.0;
			double angleRad = angleDeg * Math::PI / 180.0;
			// Split the distance into how far forward (x) and how far left (y).
			x_[i] = rangeM * Math::Sin(angleRad);
			y_[i] = -rangeM * Math::Cos(angleRad);
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

// Ask for one scan, read the reply, and check it.
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

// Put the latest scan into shared memory for the other modules.
error_state LiDAR::processSharedMemory() {
	if (SM_Laser_ == nullptr) {
		return error_state::ERR_SM;
	}
	Monitor::Enter(SM_Laser_->lockObject);
	try {
		Array::Copy(x_, SM_Laser_->x, STANDARD_LASER_LENGTH);
		Array::Copy(y_, SM_Laser_->y, STANDARD_LASER_LENGTH);
	}
	finally {
		Monitor::Exit(SM_Laser_->lockObject);
	}
	return error_state::SUCCESS;
}

// Print the 361 points as x,y pairs, four to a line.
void LiDAR::printScan() {
	// Build the whole block first, then print it in one go, so other messages can't land in the middle.
	Text::StringBuilder^ text = gcnew Text::StringBuilder();
	text->AppendLine("LiDAR scan #" + scanCount_ + "  (" + STANDARD_LASER_LENGTH + " points, metres, x = forward, y = left)");
	for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
		text->Append(String::Format("{0,3}: ({1,6:F2}, {2,6:F2})   ", i, x_[i], y_[i]));
		if (i % 4 == 3 || i == STANDARD_LASER_LENGTH - 1) {
			text->AppendLine();
		}
	}
	Console::Write(text->ToString());
}

// Tell TMM this thread is still running.
void LiDAR::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_LASER;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

// True when TMM tells this module to stop, or when TMM itself has stopped.
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

// The LiDAR thread: connect, wait for the others, then keep reading scans until told to stop.
void LiDAR::threadFunction() {
	Console::WriteLine("LiDAR: thread started.");

	error_state status = connect(WEEDER_ADDRESS, LIDAR_PORT);
	if (status != error_state::SUCCESS) {
		printError(status);
	}

	// Check in with the other threads even if connecting failed, so they aren't left waiting.
	waitForStartBarrier();

	if (status != error_state::SUCCESS) {
		Console::WriteLine("LiDAR: cannot run without a connection - thread stopping (TMM will notice the missing heartbeat).");
		return;
	}

	try {
		while (!getShutdownFlag()) {
			status = communicate();
			heartbeat();								// Still running, even if this scan was bad

			if (status == error_state::SUCCESS) {
				processSharedMemory();
				scanCount_++;
				if (scanCount_ % PRINT_EVERY_N_SCANS == 0) {
					printScan();
				}
			}
			else {
				printError(status);
				if (status == error_state::ERR_CONNECTION) {
					break;									// Lost the LiDAR: stop and let TMM deal with it
				}
			}
			Thread::Sleep(SCAN_PERIOD_MS);					// The LiDAR only makes 50 scans a second
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("LiDAR: unexpected error - " + e->Message);
	}

	// Close the connection. Any error here doesn't matter, since the thread is ending anyway.
	try {
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
	}
	catch (Exception^) {}
	Console::WriteLine("LiDAR: thread shut down after " + scanCount_ + " scans.");
}
