#pragma once
/*****************
UGVModule.h
ALEXANDER CUNIO 2024
*****************/

/*
This file defines the base class for ALL UGV modules (TMM, Laser, GPS, Vehicle Control, Display, Controller).
Basic classes (TMM, Controller) will be derived directly from this class, whereas those that require networking
to external devices (GPS, Laser, VC, Display) will derive from the networked module class (which itself inherits from this class).
For clarification of inheritance requirements see diagram in the assignment specification.
*/

#include <iostream>
#include <SMObjects.h>

// ERROR HANDLING. Use this as the return value in your functions
enum error_state {
	SUCCESS,
	ERR_STARTUP,
	ERR_NO_DATA,
	ERR_INVALID_DATA,
	ERR_SM,
	ERR_CONNECTION,
	// Additional error types defined for this solution
	ERR_AUTHENTICATION,		// Server rejected the zID authentication
	ERR_TIMEOUT				// Expected data did not arrive in time
};

ref class UGVModule abstract
{
	public:
		/**
		 * Send/Recieve data from shared memory structures
		 *
		 * @returns the success of this function at completion
		*/
		virtual error_state processSharedMemory() = 0;

		/**
		 * Get Shutdown signal for the current, from Process Management SM
		 *
		 * @returns the current state of the shutdown flag
		*/
		virtual bool getShutdownFlag() = 0;

		/**
		 * Main runner for the thread to perform all required actions
		*/
		virtual void threadFunction() = 0;

		/**
		 * A helper function for printing a helpful error message based on the error type
		 * returned from a function
		 *
		 * @param error the error that should be printed out
		*/
		static void printError(error_state error)
		{
			switch (error)
			{
				case SUCCESS:
					std::cout << "Success." << std::endl;
					break;
				case ERR_STARTUP:
					std::cout << "ERROR: Module failed to start." << std::endl;
					break;
				case ERR_NO_DATA:
					std::cout << "ERROR: No Data Available." << std::endl;
					break;
				case ERR_INVALID_DATA:
					std::cout << "ERROR: Invalid Data Received." << std::endl;
					break;
				case ERR_SM:
					std::cout << "ERROR: Shared memory access failed." << std::endl;
					break;
				case ERR_CONNECTION:
					std::cout << "ERROR: Network connection failed." << std::endl;
					break;
				case ERR_AUTHENTICATION:
					std::cout << "ERROR: Authentication rejected by server." << std::endl;
					break;
				case ERR_TIMEOUT:
					std::cout << "ERROR: Timed out waiting for data." << std::endl;
					break;
				default:
					std::cout << "ERROR: Unknown error code " << static_cast<int>(error) << "." << std::endl;
					break;
			}
		}

	protected:
		SM_ThreadManagement^ SM_TM_;

		// Helpers shared by every module (added for this solution).

		// Wait at TMM's start-up barrier so that all threads begin their main loops together.
		// After start-up TMM removes the barrier, so a restarted thread skips straight past.
		void waitForStartBarrier()
		{
			Barrier^ barrier = nullptr;
			Monitor::Enter(SM_TM_->lockObject);
			try { barrier = SM_TM_->ThreadBarrier; }
			finally { Monitor::Exit(SM_TM_->lockObject); }

			if (barrier != nullptr) {
				barrier->SignalAndWait(BARRIER_TIMEOUT_MS);
			}
		}

		// True when TMM has stopped refreshing its own stopwatch, i.e. TMM is no longer running.
		// Every module then shuts itself down rather than carrying on unsupervised.
		bool tmmHasStopped()
		{
			bool stopped = false;
			Monitor::Enter(SM_TM_->lockObject);
			try { stopped = SM_TM_->WatchList[bitIndex(bit_TM)]->ElapsedMilliseconds > TM_ALIVE_TIMEOUT_MS; }
			finally { Monitor::Exit(SM_TM_->lockObject); }
			return stopped;
		}
};
