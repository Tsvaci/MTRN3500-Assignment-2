#include "CrashAvoidance.h"

CrashAvoidance::CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser, SM_VehicleControl^ SM_VC, SM_CrashAvoidance^ SM_CA) {
	SM_TM_ = SM_TM;
	SM_Laser_ = SM_Laser;
	SM_VC_ = SM_VC;
	SM_CA_ = SM_CA;
	scanX_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	scanY_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	requestedSpeed_ = 0.0;
	requestedSteering_ = 0.0;
	lastScanNumber_ = 0;
	timeSinceNewScan_ = Stopwatch::StartNew();
	scanStale_ = true;
	maxForwardSpeed_ = 0.0;
	maxLeftSteer_ = 0.0;
	maxRightSteer_ = 0.0;
	drivenPathClearance_ = 0.0;
}

// Read inputs: copy the scan and the driver's command; the scan is stale if its counter
// hasn't changed for SCAN_STALE_MS (or no scan has arrived yet).
error_state CrashAvoidance::readInputs() {
	if (SM_Laser_ == nullptr || SM_VC_ == nullptr || SM_CA_ == nullptr) {
		return error_state::ERR_SM;
	}

	unsigned int scanNumber;
	Monitor::Enter(SM_Laser_->lockObject);
	try {
		scanNumber = SM_Laser_->scanNumber;
		Array::Copy(SM_Laser_->x, scanX_, STANDARD_LASER_LENGTH);
		Array::Copy(SM_Laser_->y, scanY_, STANDARD_LASER_LENGTH);
	}
	finally {
		Monitor::Exit(SM_Laser_->lockObject);
	}

	Monitor::Enter(SM_VC_->lockObject);
	try {
		requestedSpeed_ = SM_VC_->Speed;
		requestedSteering_ = SM_VC_->Steering;
	}
	finally {
		Monitor::Exit(SM_VC_->lockObject);
	}

	if (scanNumber != lastScanNumber_) {
		lastScanNumber_ = scanNumber;
		timeSinceNewScan_->Restart();
	}
	bool stale = (scanNumber == 0) || (timeSinceNewScan_->ElapsedMilliseconds > SCAN_STALE_MS);
	if (stale && !scanStale_) {
		Console::WriteLine("CrashAvoidance: LiDAR data stale - forward motion stopped.");
	}
	else if (!stale && scanStale_) {
		Console::WriteLine("CrashAvoidance: LiDAR data OK - checking paths.");
	}
	scanStale_ = stale;
	return error_state::SUCCESS;
}

// Predict the path: how far the vehicle travels (at the rear axle) before its body reaches the
// point, or -1 if it would miss it.
// Straight: a corridor of half-width + margin each side. Turning: the vehicle circles a centre
// level with the rear axle, turn radius = wheelbase / tan(steer), and its front edge and inner
// side sweep a ring around that centre.
// Frame: x forward, y left, origin at the LiDAR (front centre). Positive steer = left.
double CrashAvoidance::distanceAlongPath(double pointX, double pointY, double steerDeg) {
	double halfWidth = VEHICLE_HALF_WIDTH_M + SIDE_MARGIN_M;
	double lidarToRearAxle = LIDAR_AHEAD_OF_REAR_AXLE_M;

	if (Math::Abs(steerDeg) < 0.5) {
		if (pointX < 0.0 || Math::Abs(pointY) > halfWidth) {
			return -1.0;
		}
		return pointX;
	}

	// Work as if turning left; a right turn is the mirror image (flip y).
	double turnRadius = Math::Abs(WHEELBASE_M / Math::Tan(steerDeg * Math::PI / 180.0));
	double mirroredY = (steerDeg > 0.0) ? pointY : -pointY;

	// Turn centre at (-lidarToRearAxle, turnRadius). The ring runs from the inner side
	// (radius turnRadius - halfWidth) out to the outer front corner.
	double pointFromCentreX = pointX + lidarToRearAxle;
	double pointFromCentreY = mirroredY - turnRadius;
	double pointRadius = Math::Sqrt(pointFromCentreX * pointFromCentreX + pointFromCentreY * pointFromCentreY);
	double innerRadius = Math::Max(0.0, turnRadius - halfWidth);
	double outerRadius = Math::Sqrt(lidarToRearAxle * lidarToRearAxle + (turnRadius + halfWidth) * (turnRadius + halfWidth));
	if (pointRadius < innerRadius || pointRadius > outerRadius) {
		return -1.0;
	}

	// The part of the body that will touch the point: the front edge if it reaches that radius,
	// otherwise the inner side.
	double contactX, contactY;
	double innerCornerRadius = Math::Sqrt(lidarToRearAxle * lidarToRearAxle + (turnRadius - halfWidth) * (turnRadius - halfWidth));
	if (pointRadius >= innerCornerRadius) {
		contactX = 0.0;
		contactY = turnRadius - Math::Sqrt(pointRadius * pointRadius - lidarToRearAxle * lidarToRearAxle);
	}
	else {
		contactX = -lidarToRearAxle + Math::Sqrt(Math::Max(0.0, pointRadius * pointRadius - (turnRadius - halfWidth) * (turnRadius - halfWidth)));
		contactY = halfWidth;
	}

	// Angle the vehicle turns (anticlockwise) to bring that body point onto the obstacle.
	double contactFromCentreX = contactX + lidarToRearAxle;
	double contactFromCentreY = contactY - turnRadius;
	double turnAngle = Math::Atan2(contactFromCentreX * pointFromCentreY - contactFromCentreY * pointFromCentreX,
		contactFromCentreX * pointFromCentreX + contactFromCentreY * pointFromCentreY);
	if (turnAngle < 0.0) {
		turnAngle += 2.0 * Math::PI;
	}
	return turnRadius * turnAngle;
}

// Measure clearance: free distance along the path for a steering angle, capped at LOOKAHEAD_M.
// Uses the second-nearest point so a single stray reading can't stop the vehicle.
double CrashAvoidance::pathClearance(double steerDeg) {
	double nearest = LOOKAHEAD_M;
	double secondNearest = LOOKAHEAD_M;

	for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
		double range = Math::Sqrt(scanX_[i] * scanX_[i] + scanY_[i] * scanY_[i]);
		if (range < MIN_VALID_RANGE_M) {
			continue;                                      // No echo
		}
		double distance = distanceAlongPath(scanX_[i], scanY_[i], steerDeg);
		if (distance < 0.0) {
			continue;                                      // Not on this path
		}
		if (distance < nearest) {
			secondNearest = nearest;
			nearest = distance;
		}
		else if (distance < secondNearest) {
			secondNearest = distance;
		}
	}
	return secondNearest;
}

// Limit speed: full speed beyond CAUTION_DISTANCE_M, zero at STOP_DISTANCE_M, and an even
// braking curve in between (v^2 proportional to distance left). Faster requests set aside
// more reaction distance, so the vehicle starts slowing earlier.
double CrashAvoidance::speedLimitFor(double clearance) {
	double reactionDistance = Math::Max(0.0, requestedSpeed_) * REACTION_TIME_S;
	double brakingDistance = clearance - reactionDistance - STOP_DISTANCE_M;
	if (brakingDistance <= 0.0) {
		return 0.0;
	}
	double fractionOfSlowingZone = brakingDistance / (CAUTION_DISTANCE_M - STOP_DISTANCE_M);
	return MAX_SPEED_MPS * Math::Min(1.0, Math::Sqrt(fractionOfSlowingZone));
}

// Ease steering, then limit speed.
// 1. Steering: if straight ahead is clear for CAUTION_DISTANCE_M, each side may steer up to the
//    sharpest angle (5 deg steps) whose path is also clear that far. If straight ahead is
//    blocked, steering is left free so the driver can steer out.
// 2. Speed: from the clearance along the path the vehicle will actually take.
void CrashAvoidance::computeLimits() {
	if (scanStale_) {
		maxForwardSpeed_ = 0.0;
		maxLeftSteer_ = 0.0;
		maxRightSteer_ = 0.0;
		drivenPathClearance_ = 0.0;
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

	double steeringAfterLimits = Math::Max(-maxRightSteer_, Math::Min(maxLeftSteer_, requestedSteering_));
	drivenPathClearance_ = pathClearance(steeringAfterLimits);
	maxForwardSpeed_ = speedLimitFor(drivenPathClearance_);
}

// Publish: write the limits and a timestamp to SM_CrashAvoidance for VC.
void CrashAvoidance::publishLimits() {
	Monitor::Enter(SM_CA_->lockObject);
	try {
		SM_CA_->maxForwardSpeed = maxForwardSpeed_;
		SM_CA_->maxLeftSteer = maxLeftSteer_;
		SM_CA_->maxRightSteer = maxRightSteer_;
		SM_CA_->pathClearance = drivenPathClearance_;
		SM_CA_->lastUpdate->Restart();
	}
	finally {
		Monitor::Exit(SM_CA_->lockObject);
	}
}

// One cycle: read inputs, work out the limits, publish them.
error_state CrashAvoidance::processSharedMemory() {
	error_state result = readInputs();
	if (result != error_state::SUCCESS) {
		return result;
	}
	computeLimits();
	publishLimits();
	return error_state::SUCCESS;
}

// Tell TMM this thread is alive.
void CrashAvoidance::heartbeat() {
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->heartbeat |= bit_CRASHAVOIDANCE;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

// True if TMM asked this thread to stop, or TMM itself has stopped.
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

// Main loop, every 20 ms. Non-critical: if this thread dies, its limits go stale, VC holds the
// vehicle, and TMM restarts it.
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
