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

// ---------------------------------------------------------------------------------------------
// Open the TCP connection and authenticate with the zID.
// ---------------------------------------------------------------------------------------------
error_state LiDAR::connect(String^ hostName, int portNumber) {
	try {
		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;
		Client->ReceiveTimeout = 2000;
		Client->SendTimeout = 1000;
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

// Service details, Section 2: send "<zID digits>\n", expect "OK\n".
error_state LiDAR::authenticate() {
	try {
		array<unsigned char>^ request = Encoding::ASCII->GetBytes(STUDENT_ID "\n");
		Stream->Write(request, 0, request->Length);

		int received = Stream->Read(ReadData, 0, ReadData->Length);
		if (received <= 0) {
			return error_state::ERR_NO_DATA;
		}
		String^ reply = Encoding::ASCII->GetString(ReadData, 0, received);
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

// Service details, Section 3: <STX>sRN LMDscandata<ETX> asks for the most recent scan.
error_state LiDAR::requestScan() {
	try {
		array<unsigned char>^ body = Encoding::ASCII->GetBytes("sRN LMDscandata");
		array<unsigned char>^ command = gcnew array<unsigned char>(body->Length + 2);
		command[0] = 0x02;										// STX
		Array::Copy(body, 0, command, 1, body->Length);
		command[command->Length - 1] = 0x03;					// ETX
		Stream->Write(command, 0, command->Length);
	}
	catch (Exception^ e) {
		Console::WriteLine("LiDAR: failed to send scan request - " + e->Message);
		return error_state::ERR_CONNECTION;
	}
	return error_state::SUCCESS;
}

// Read bytes until the ETX terminator arrives, then hand back the ASCII between STX and ETX.
error_state LiDAR::readTelegram(String^% telegram) {
	int total = 0;
	try {
		while (true) {
			if (total >= ReadData->Length) {
				Console::WriteLine("LiDAR: telegram larger than the read buffer - discarded.");
				return error_state::ERR_INVALID_DATA;
			}
			int received = Stream->Read(ReadData, total, ReadData->Length - total);
			if (received <= 0) {
				return error_state::ERR_CONNECTION;			// Server closed the connection
			}
			total += received;
			if (Array::IndexOf(ReadData, (unsigned char)0x03, total - received, received) >= 0) {
				break;
			}
		}
	}
	catch (IO::IOException^) {
		return error_state::ERR_TIMEOUT;						// ReceiveTimeout elapsed
	}
	catch (Exception^ e) {
		Console::WriteLine("LiDAR: read error - " + e->Message);
		return error_state::ERR_CONNECTION;
	}

	int start = Array::IndexOf(ReadData, (unsigned char)0x02, 0, total);
	int end = Array::IndexOf(ReadData, (unsigned char)0x03, 0, total);
	if (start < 0 || end < 0 || end <= start) {
		return error_state::ERR_INVALID_DATA;
	}
	telegram = Encoding::ASCII->GetString(ReadData, start + 1, end - start - 1);
	return error_state::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// Decode "sRA LMDscandata ..." (SICK LMS1xx manual, section 10.2.4). Fields are space-separated
// hexadecimal. After the token DIST1 come: ScalingFactor (IEEE float as hex), ScalingOffset,
// StartingAngle (1/10000 deg, signed), AngularStepWidth (1/10000 deg), NumberData, then the
// distances in mm. Converted here to metres in the vehicle base frame: x forward, y to the left.
// Service details Figure 1: 0 deg is the right-hand side, 90 deg straight ahead, 180 deg the left.
// The real LMS151 answers "sRA"; the Weeder simulator answers "sRN" (echoing the request), so accept both.
// A distance of 0 means no echo; it is stored as (0, 0) and must be ignored by consumers.
// ---------------------------------------------------------------------------------------------
error_state LiDAR::parseScan(String^ telegram) {
	array<String^>^ fields = telegram->Split(' ');

	if (fields->Length < 3 || (fields[0] != "sRA" && fields[0] != "sRN") || fields[1] != "LMDscandata") {
		Console::WriteLine("LiDAR: unexpected reply: " + (telegram->Length > 40 ? telegram->Substring(0, 40) : telegram));
		return error_state::ERR_INVALID_DATA;
	}

	int dist = Array::IndexOf(fields, "DIST1");
	if (dist < 0 || dist + 6 > fields->Length) {
		return error_state::ERR_INVALID_DATA;
	}

	try {
		unsigned int scaleBits = Convert::ToUInt32(fields[dist + 1], 16);
		float scale = BitConverter::ToSingle(BitConverter::GetBytes(scaleBits), 0);
		int startAngle = Convert::ToInt32(fields[dist + 3], 16);			// 1/10000 degree, two's complement
		unsigned int step = Convert::ToUInt16(fields[dist + 4], 16);		// 1/10000 degree
		int count = Convert::ToUInt16(fields[dist + 5], 16);

		if (count != STANDARD_LASER_LENGTH || dist + 6 + count > fields->Length) {
			Console::WriteLine("LiDAR: expected " + STANDARD_LASER_LENGTH + " points, telegram has " + count);
			return error_state::ERR_INVALID_DATA;
		}
		if (scale <= 0.0f || step == 0) {
			return error_state::ERR_INVALID_DATA;
		}

		for (int i = 0; i < count; i++) {
			unsigned int raw = Convert::ToUInt16(fields[dist + 6 + i], 16);
			double range = raw * scale / 1000.0;								// mm -> m
			double angle = (startAngle + (double)i * step) / 10000.0 * Math::PI / 180.0;
			x_[i] = range * Math::Sin(angle);
			y_[i] = -range * Math::Cos(angle);
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

// One request/reply cycle, including validation.
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

// Copy the latest scan into shared memory under the lock.
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

// Progress check item (ii): the 361 points as x,y pairs, four per line.
void LiDAR::printScan() {
	Text::StringBuilder^ sb = gcnew Text::StringBuilder();
	sb->AppendLine("LiDAR scan #" + scanCount_ + "  (" + STANDARD_LASER_LENGTH + " points, metres, x = forward, y = left)");
	for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
		sb->Append(String::Format("{0,3}: ({1,6:F2}, {2,6:F2})   ", i, x_[i], y_[i]));
		if (i % 4 == 3 || i == STANDARD_LASER_LENGTH - 1) {
			sb->AppendLine();
		}
	}
	Console::Write(sb->ToString());
}

void LiDAR::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_LASER;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

// Shut down if TMM says so, or if TMM itself has stopped beating.
bool LiDAR::getShutdownFlag() {
	bool flag = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		flag = (SM_TM_->shutdown & bit_LASER) != 0
			|| SM_TM_->WatchList[bitIndex(bit_TM)]->ElapsedMilliseconds > TM_ALIVE_TIMEOUT_MS;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return flag;
}

// ---------------------------------------------------------------------------------------------
// The LiDAR thread: connect, wait for the others, then request/convert/publish until told to stop.
// ---------------------------------------------------------------------------------------------
void LiDAR::threadFunction() {
	Console::WriteLine("LiDAR: thread started.");

	error_state status = connect(WEEDER_ADDRESS, LIDAR_PORT);
	if (status != error_state::SUCCESS) {
		printError(status);
	}

	// Join the start-up barrier whether or not the connection worked, so the others are not held up.
	Barrier^ barrier = nullptr;
	Monitor::Enter(SM_TM_->lockObject);
	try { barrier = SM_TM_->ThreadBarrier; }
	finally { Monitor::Exit(SM_TM_->lockObject); }
	if (barrier != nullptr) {
		barrier->SignalAndWait(BARRIER_TIMEOUT_MS);
	}

	if (status != error_state::SUCCESS) {
		Console::WriteLine("LiDAR: cannot run without a connection - thread stopping (TMM will notice the missing heartbeat).");
		return;
	}

	try {
		while (!getShutdownFlag()) {
			status = communicate();
			heartbeat();								// Alive even if this particular scan was bad

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
					break;									// Lost the server: stop, and let TMM act
				}
			}
			Thread::Sleep(20);								// Sensor runs at 50 Hz; no point asking faster
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
