/*
Main.cpp
Starts the TMM thread. TMM creates the shared memory and all other threads.
*/
#include "TMM.h"

int main(array<String^>^ args) {
	ThreadManagement^ TMM = gcnew ThreadManagement();

	error_state result = TMM->setupSharedMemory();
	if (result != SUCCESS) {
		UGVModule::printError(result);
		return 1;
	}

	Thread^ TMMThread = gcnew Thread(gcnew ThreadStart(TMM, &ThreadManagement::threadFunction));
	TMMThread->Start();
	TMMThread->Join();

	if (TMM->endedWithCriticalFailure()) {
		Console::WriteLine("Program ended after a critical failure.");
		return 2;
	}
	Console::WriteLine("Program complete.");
	return 0;
}
