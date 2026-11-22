#include "GNSS.h"
#include "ConsoleView.h"

GNSS::GNSS(SM_ThreadManagement^ SM_TM, SM_GPS^ SM_GPS) {
	SM_TM_ = SM_TM;
	SM_GPS_ = SM_GPS;
	bufferedBytes_ = 0;
	northing_ = 0.0;
	easting_ = 0.0;
	height_ = 0.0;
	receivedCRC_ = 0;
	calculatedCRC_ = 0;
	recordCount_ = 0;
	printTimer_ = Stopwatch::StartNew();
	rejectedCount_ = 0;
	Client = nullptr;
	Stream = nullptr;
	ReadData = nullptr;
}

// No authentication: the receiver streams records as soon as we connect.
error_state GNSS::connect(String^ hostName, int portNumber) {
	try {
		Client = gcnew TcpClient(hostName, portNumber);
		Client->ReceiveTimeout = RECEIVE_TIMEOUT_MS;
		Stream = Client->GetStream();
		ReadData = gcnew array<unsigned char>(READ_BUFFER_SIZE);
	}
	catch (Exception^ e) {
		Console::WriteLine("GNSS: could not connect to " + hostName + ":" + portNumber + " (" + e->Message + ")");
		return error_state::ERR_CONNECTION;
	}
	Console::WriteLine("GNSS: connected to " + hostName + ":" + portNumber);
	return error_state::SUCCESS;
}

// Records can be split across reads or arrive several at once, so bytes are kept in
// ReadData until a whole 112-byte record is there.
error_state GNSS::communicate() {
	try {
		int bytesReceived = Stream->Read(ReadData, bufferedBytes_, ReadData->Length - bufferedBytes_);
		if (bytesReceived <= 0) {
			return error_state::ERR_CONNECTION;
		}
		bufferedBytes_ += bytesReceived;
	}
	catch (IO::IOException^) {
		return error_state::ERR_TIMEOUT;
	}
	catch (Exception^ e) {
		Console::WriteLine("GNSS: read error - " + e->Message);
		return error_state::ERR_CONNECTION;
	}

	error_state result = error_state::ERR_NO_DATA;
	while (true) {
		int headerIndex = findHeader();
		if (headerIndex < 0) {
			discardBytes(bufferedBytes_ > 2 ? bufferedBytes_ - 2 : 0);   // Keep a possible partial sync
			break;
		}
		discardBytes(headerIndex);
		if (bufferedBytes_ < RECORD_LENGTH) {
			break;                                                       // Wait for the rest
		}

		error_state recordResult = decodeRecord(0);
		if (recordResult == error_state::SUCCESS) {
			discardBytes(RECORD_LENGTH);
			result = error_state::SUCCESS;
		}
		else {
			discardBytes(1);                                             // False sync or bad record: search again
			if (result != error_state::SUCCESS) {
				result = recordResult;
			}
		}
	}

	return result;
}

int GNSS::findHeader() {
	for (int i = 0; i + 2 < bufferedBytes_; i++) {
		if (ReadData[i] == 0xAA && ReadData[i + 1] == 0x44 && ReadData[i + 2] == 0x12) {
			return i;
		}
	}
	return -1;
}

// Header: byte 3 = header length, bytes 4-5 = message ID, bytes 8-9 = body length.
// BESTUTM body: Northing, Easting, Height are doubles at 16, 24, 32. CRC32 follows the body.
error_state GNSS::decodeRecord(int offset) {
	int headerLength = ReadData[offset + 3];
	unsigned short messageId = BitConverter::ToUInt16(ReadData, offset + 4);
	unsigned short bodyLength = BitConverter::ToUInt16(ReadData, offset + 8);

	if (headerLength != HEADER_LENGTH || messageId != BESTUTM_MESSAGE_ID || bodyLength != BODY_LENGTH) {
		return error_state::ERR_INVALID_DATA;
	}

	int crcIndex = offset + HEADER_LENGTH + BODY_LENGTH;
	receivedCRC_ = BitConverter::ToUInt32(ReadData, crcIndex);
	{
		pin_ptr<unsigned char> record = &ReadData[offset];   // Stops the GC moving the array during the native call
		calculatedCRC_ = CalculateBlockCRC32(HEADER_LENGTH + BODY_LENGTH, record);
	}

	if (calculatedCRC_ != receivedCRC_) {
		printPosition(false);
		return error_state::ERR_INVALID_DATA;
	}

	int body = offset + HEADER_LENGTH;
	northing_ = BitConverter::ToDouble(ReadData, body + 16);
	easting_ = BitConverter::ToDouble(ReadData, body + 24);
	height_ = BitConverter::ToDouble(ReadData, body + 32);
	recordCount_++;
	printPosition(true);
	return error_state::SUCCESS;
}

void GNSS::discardBytes(int count) {
	if (count <= 0) {
		return;
	}
	if (count >= bufferedBytes_) {
		bufferedBytes_ = 0;
		return;
	}
	Array::Copy(ReadData, count, ReadData, 0, bufferedBytes_ - count);
	bufferedBytes_ -= count;
}

error_state GNSS::processSharedMemory() {
	if (SM_GPS_ == nullptr) {
		return error_state::ERR_SM;
	}
	Monitor::Enter(SM_GPS_->lockObject);
	try {
		SM_GPS_->Northing = northing_;
		SM_GPS_->Easting = easting_;
		SM_GPS_->Height = height_;
	}
	finally {
		Monitor::Exit(SM_GPS_->lockObject);
	}
	return error_state::SUCCESS;
}

// Good records: the latest one every 2 s (the receiver sends several a second).
// CRC mismatches: always, straight away.
void GNSS::printPosition(bool crcOk) {
	if (!crcOk) {
		rejectedCount_++;
	}
	if (crcOk && ConsoleView::isActive()) {
		if (printTimer_->ElapsedMilliseconds < VIEW_PERIOD_MS) {
			return;
		}
		printTimer_->Restart();
		ConsoleView::set("gnss", String::Format("Northing {0:F3}    Easting {1:F3}    Height {2:F3}", northing_, easting_, height_), ConsoleColor::White);
		ConsoleView::set("gnsscrc", String::Format("calc {0:X8}   recv {1:X8}   OK    ({2} good, {3} rejected)",
			calculatedCRC_, receivedCRC_, recordCount_, rejectedCount_), rejectedCount_ == 0 ? ConsoleColor::Green : ConsoleColor::DarkYellow);
		return;
	}
	if (crcOk) {
		if (printTimer_->ElapsedMilliseconds < PRINT_PERIOD_MS) {
			return;
		}
		printTimer_->Restart();
		Console::WriteLine(String::Format("GNSS: Northing {0,12:F3}  Easting {1,11:F3}  Height {2,8:F3}  CRC calc {3:X8} recv {4:X8} OK",
			northing_, easting_, height_, calculatedCRC_, receivedCRC_));
	}
	else {
		Console::WriteLine(String::Format("GNSS: CRC MISMATCH - calc {0:X8} recv {1:X8}, record discarded", calculatedCRC_, receivedCRC_));
	}
}

void GNSS::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_GPS;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

bool GNSS::getShutdownFlag() {
	bool shutdownRequested = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdownRequested = (SM_TM_->shutdown & bit_GPS) != 0;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdownRequested || tmmHasStopped();
}

// Non-critical: on any failure the thread just ends, and TMM restarts it.
void GNSS::threadFunction() {
	Console::WriteLine("GNSS: thread started.");

	error_state status = connect(WEEDER_ADDRESS, GNSS_PORT);
	if (status != error_state::SUCCESS) {
		printError(status);
	}

	waitForStartBarrier();

	if (status != error_state::SUCCESS) {
		Console::WriteLine("GNSS: no connection - thread stopping (TMM will restart it).");
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
			}
			else if (status == error_state::ERR_CONNECTION) {
				printError(status);
				break;
			}
			else if (status == error_state::ERR_TIMEOUT) {
				if (++consecutiveTimeouts >= MAX_CONSECUTIVE_TIMEOUTS) {
					Console::WriteLine("GNSS: no data for " + consecutiveTimeouts + " s - reconnecting.");
					break;
				}
			}
			// ERR_NO_DATA: only part of a record so far; ERR_INVALID_DATA: already reported.
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("GNSS: unexpected error - " + e->Message);
	}

	try {
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
	}
	catch (Exception^) {}
	Console::WriteLine("GNSS: thread shut down after " + recordCount_ + " records.");
}

// Provided CRC32 code. The spec's version leaves the else branch empty; this is the corrected form.
unsigned long GNSS::CRC32Value(int i) {
	unsigned long ulCRC = i;
	for (int j = 8; j > 0; j--) {
		if (ulCRC & 1)
			ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return ulCRC;
}

unsigned long GNSS::CalculateBlockCRC32(unsigned long ulCount, unsigned char* ucBuffer) {
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while (ulCount-- != 0) {
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value(((int)ulCRC ^ *ucBuffer++) & 0xff);
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return ulCRC;
}
