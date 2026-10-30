/*
Main.cpp
Entry point. The only thread created here is Thread Management; it creates the shared memory
and every other thread (see Section 5 of the specification).
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

	Console::WriteLine("Program complete.");
	return 0;
}
