#include "CrashAvoidance.h"

CrashAvoidance::CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser, SM_VehicleControl^ SM_VC, SM_CrashAvoidance^ SM_CA) {
	SM_TM_ = SM_TM;
	SM_Laser_ = SM_Laser;
	SM_VC_ = SM_VC;
	SM_CA_ = SM_CA;
	x_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	y_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	speed_ = 0.0;
	steering_ = 0.0;
	lastScanNumber_ = 0;
	scanAge_ = Stopwatch::StartNew();
	scanStale_ = true;
	maxForwardSpeed_ = 0.0;
	maxLeftSteer_ = 0.0;
	maxRightSteer_ = 0.0;
	pathClearance_ = 0.0;
}

// Copies the latest scan and the driver's command. A scan counts as stale if no new one has
// arrived for SCAN_STALE_MS (or none has arrived yet).
error_state CrashAvoidance::readInputs() {
	if (SM_Laser_ == nullptr || SM_VC_ == nullptr || SM_CA_ == nullptr) {
		return error_state::ERR_SM;
	}

	unsigned int scanNumber;
	Monitor::Enter(SM_Laser_->lockObject);
	try {
		scanNumber = SM_Laser_->scanNumber;
		Array::Copy(SM_Laser_->x, x_, STANDARD_LASER_LENGTH);
		Array::Copy(SM_Laser_->y, y_, STANDARD_LASER_LENGTH);
	}
	finally {
		Monitor::Exit(SM_Laser_->lockObject);
	}

	Monitor::Enter(SM_VC_->lockObject);
	try {
		speed_ = SM_VC_->Speed;
		steering_ = SM_VC_->Steering;
	}
	finally {
		Monitor::Exit(SM_VC_->lockObject);
	}

	if (scanNumber != lastScanNumber_) {
		lastScanNumber_ = scanNumber;
		scanAge_->Restart();
	}
	bool stale = (scanNumber == 0) || (scanAge_->ElapsedMilliseconds > SCAN_STALE_MS);
	if (stale && !scanStale_) {
		Console::WriteLine("CrashAvoidance: LiDAR data stale - forward motion stopped.");
	}
	else if (!stale && scanStale_) {
		Console::WriteLine("CrashAvoidance: LiDAR data OK - checking paths.");
	}
	scanStale_ = stale;
	return error_state::SUCCESS;
}

// The vehicle is modelled as driving along an arc set by the steering angle (radius
// = wheelbase / tan(steer)), sweeping a corridor as wide as the vehicle plus a margin.
// Returns how far along that arc the point is, or -1 if the vehicle would miss it.
// Frame: x forward, y left, origin at the LiDAR (front centre). Positive steer = left.
double CrashAvoidance::distanceAlongPath(double px, double py, double steerDeg) {
	double corridorHalfWidth = VEHICLE_HALF_WIDTH_M + SIDE_MARGIN_M;

	if (Math::Abs(steerDeg) < 0.5) {
		if (px < 0.0 || Math::Abs(py) > corridorHalfWidth) {
			return -1.0;
		}
		return px;
	}

	// Turn centre is level with the LiDAR, R to the left (R > 0) or right (R < 0).
	double R = WHEELBASE_M / Math::Tan(steerDeg * Math::PI / 180.0);
	double dx = px;
	double dy = py - R;
	double radiusOfPoint = Math::Sqrt(dx * dx + dy * dy);
	if (Math::Abs(radiusOfPoint - Math::Abs(R)) > corridorHalfWidth) {
		return -1.0;
	}

	// Angle the vehicle turns through to reach the point, measured in its direction of travel.
	double turnAngle = Math::Atan2(dx, -dy * Math::Sign(R));
	if (turnAngle < 0.0) {
		return -1.0;
	}
	return Math::Abs(R) * turnAngle;
}

// Free distance along the path for a given steering angle, capped at LOOKAHEAD_M.
// Uses the second-nearest point on the path so a single stray reading can't stop the vehicle.
double CrashAvoidance::pathClearance(double steerDeg) {
	double nearest = LOOKAHEAD_M;
	double secondNearest = LOOKAHEAD_M;

	for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
		double range = Math::Sqrt(x_[i] * x_[i] + y_[i] * y_[i]);
		if (range < MIN_VALID_RANGE_M) {
			continue;                                      // No echo
		}
		double s = distanceAlongPath(x_[i], y_[i], steerDeg);
		if (s < 0.0) {
			continue;
		}
		if (s < nearest) {
			secondNearest = nearest;
			nearest = s;
		}
		else if (s < secondNearest) {
			secondNearest = s;
		}
	}
	return secondNearest;
}

// Full speed beyond CAUTION_DISTANCE_M, zero at STOP_DISTANCE_M, and in between the speed from
// which the vehicle can still brake evenly to a halt at the stop distance (v^2 proportional to
// distance left). The faster the driver is asking to go, the more distance is set aside for the
// reaction time, so the vehicle reacts earlier at higher speed.
double CrashAvoidance::speedLimitFor(double clearance) {
	double reactionDistance = Math::Max(0.0, speed_) * REACTION_TIME_S;
	double usable = clearance - reactionDistance - STOP_DISTANCE_M;
	if (usable <= 0.0) {
		return 0.0;
	}
	double fraction = usable / (CAUTION_DISTANCE_M - STOP_DISTANCE_M);
	return MAX_SPEED_MPS * Math::Min(1.0, Math::Sqrt(fraction));
}

// 1. Steering limits: starting from straight ahead, allow each side up to the sharpest angle
//    (in 5 deg steps) whose path is clear for CAUTION_DISTANCE_M. Turning further would take
//    the vehicle into something within 1 m, so the turn is eased back to that angle.
//    If straight ahead is already blocked, steering is not limited: steering away may be the
//    way out, and step 2 still limits the speed on whatever path is chosen.
// 2. Speed limit: from the clear distance along the path the vehicle will actually take
//    (the driver's steering after step 1).
void CrashAvoidance::computeLimits() {
	if (scanStale_) {
		maxForwardSpeed_ = 0.0;
		maxLeftSteer_ = 0.0;
		maxRightSteer_ = 0.0;
		pathClearance_ = 0.0;
		return;
	}

	if (pathClearance(0.0) >= CAUTION_DISTANCE_M) {
		maxLeftSteer_ = 0.0;
		for (double steer = STEER_STEP_DEG; steer <= MAX_STEER_DEG; steer += STEER_STEP_DEG) {
			if (pathClearance(steer) < CAUTION_DISTANCE_M) break;
			maxLeftSteer_ = steer;
		}
		maxRightSteer_ = 0.0;
		for (double steer = STEER_STEP_DEG; steer <= MAX_STEER_DEG; steer += STEER_STEP_DEG) {
			if (pathClearance(-steer) < CAUTION_DISTANCE_M) break;
			maxRightSteer_ = steer;
		}
	}
	else {
		maxLeftSteer_ = MAX_STEER_DEG;
		maxRightSteer_ = MAX_STEER_DEG;
	}

	double steerToCheck = Math::Max(-maxRightSteer_, Math::Min(maxLeftSteer_, steering_));
	pathClearance_ = pathClearance(steerToCheck);
	maxForwardSpeed_ = speedLimitFor(pathClearance_);
}

void CrashAvoidance::publishLimits() {
	Monitor::Enter(SM_CA_->lockObject);
	try {
		SM_CA_->maxForwardSpeed = maxForwardSpeed_;
		SM_CA_->maxLeftSteer = maxLeftSteer_;
		SM_CA_->maxRightSteer = maxRightSteer_;
		SM_CA_->pathClearance = pathClearance_;
		SM_CA_->lastUpdate->Restart();
	}
	finally {
		Monitor::Exit(SM_CA_->lockObject);
	}
}

error_state CrashAvoidance::processSharedMemory() {
	error_state result = readInputs();
	if (result != error_state::SUCCESS) {
		return result;
	}
	computeLimits();
	publishLimits();
	return error_state::SUCCESS;
}

void CrashAvoidance::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_CRASHAVOIDANCE;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

bool CrashAvoidance::getShutdownFlag() {
	bool shutdownRequested = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdownRequested = (SM_TM_->shutdown & bit_CRASHAVOIDANCE) != 0;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdownRequested || tmmHasStopped();
}

// Non-critical: if this thread dies, its limits go stale, VC holds the vehicle, and TMM restarts it.
void CrashAvoidance::threadFunction() {
	Console::WriteLine("CrashAvoidance: thread started.");

	waitForStartBarrier();

	try {
		while (!getShutdownFlag()) {
			error_state status = processSharedMemory();
			if (status != error_state::SUCCESS) {
				printError(status);
				break;
			}
			heartbeat();
			Thread::Sleep(CHECK_PERIOD_MS);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("CrashAvoidance: unexpected error - " + e->Message);
	}
	Console::WriteLine("CrashAvoidance: thread shut down.");
}
