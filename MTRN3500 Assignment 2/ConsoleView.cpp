#include "ConsoleView.h"
#include <SMObjects.h>

void ConsoleView::start() {
	if (Console::IsOutputRedirected) {
		return;                                  // Plain lines for logs and test scripts
	}
	Monitor::Enter(lock_);
	try {
		screen_ = Console::Out;
		upTime_ = Diagnostics::Stopwatch::StartNew();
		try {
			Console::Title = "UGV Tele-operation - MTRN3500 Assignment 2";
			Console::Clear();
			Console::CursorVisible = false;
		}
		catch (Exception^) {}
		Console::SetOut(gcnew EventCatcher());
		active_ = true;
	}
	finally {
		Monitor::Exit(lock_);
	}
	timer_ = gcnew Timer(gcnew TimerCallback(&ConsoleView::onTimer), nullptr, 0, REDRAW_PERIOD_MS);
}

// Final redraw, then later messages print as normal lines under the screen.
void ConsoleView::stop() {
	if (!active_) {
		return;
	}
	if (timer_ != nullptr) {
		delete timer_;                           // Stops the redraws (C++/CLI calls Dispose)
		timer_ = nullptr;
	}
	Monitor::Enter(lock_);
	try {
		render();
		active_ = false;
		Console::SetOut(screen_);
		try {
			Console::SetCursorPosition(0, lastHeight_);
			Console::CursorVisible = true;
		}
		catch (Exception^) {}
	}
	finally {
		Monitor::Exit(lock_);
	}
}

bool ConsoleView::isActive() {
	return active_;
}

void ConsoleView::set(String^ field, String^ text, ConsoleColor colour) {
	Monitor::Enter(lock_);
	try {
		text_[field] = text;
		colour_[field] = colour;
	}
	finally {
		Monitor::Exit(lock_);
	}
}

void ConsoleView::setThreads(unsigned char aliveMask, bool running) {
	Monitor::Enter(lock_);
	try {
		aliveMask_ = aliveMask;
		running_ = running;
	}
	finally {
		Monitor::Exit(lock_);
	}
}

// Builds complete lines from caught output and keeps the most recent ones, time-stamped.
void ConsoleView::addChar(wchar_t c) {
	Monitor::Enter(lock_);
	try {
		if (c == L'\r') {
			return;
		}
		if (c != L'\n') {
			partLine_->Append(c);
			return;
		}
		String^ line = partLine_->ToString()->Trim();
		partLine_->Clear();
		if (line->Length == 0) {
			return;
		}
		events_->Add(DateTime::Now.ToString("HH:mm:ss") + "  " + line);
		if (events_->Count > MAX_EVENTS) {
			events_->RemoveAt(0);
		}
	}
	finally {
		Monitor::Exit(lock_);
	}
}

void ConsoleView::onTimer(Object^ unused) {
	if (!Monitor::TryEnter(lock_)) {
		return;                                  // Previous redraw still running
	}
	try {
		if (active_) {
			render();
		}
	}
	catch (Exception^) {}                        // e.g. window being resized - just try again next time
	finally {
		Monitor::Exit(lock_);
	}
}

// Writes text padded to the width, in one colour.
void ConsoleView::writeText(String^ text, ConsoleColor colour, int width) {
	if (text->Length > width) {
		text = text->Substring(0, width);
	}
	Console::ForegroundColor = colour;
	screen_->Write(text->PadRight(width));
	Console::ResetColor();
}

// A grey label then the field's text in its colour, one screen row.
void ConsoleView::writeRow(String^ label, String^ field, int width) {
	String^ value = "waiting...";
	ConsoleColor colour = ConsoleColor::DarkGray;
	if (text_->ContainsKey(field)) {
		value = text_[field];
		colour = colour_[field];
	}
	writeText("  " + label->PadRight(14), ConsoleColor::Gray, 16);
	writeText(value, colour, width - 16);
	screen_->WriteLine();
}

// Caller holds lock_.
void ConsoleView::render() {
	int width = Math::Min(MAX_WIDTH, Console::WindowWidth - 1);
	int height = Console::WindowHeight;
	String^ rule = "  " + gcnew String(L'-', width - 4);
	Console::SetCursorPosition(0, 0);

	TimeSpan up = upTime_->Elapsed;
	String^ title = "  UGV TELE-OPERATION  |  MTRN3500 Assignment 2";
	String^ clock = String::Format("running {0:00}:{1:00}:{2:00}", (int)up.TotalHours, up.Minutes, up.Seconds);
	writeText(title, ConsoleColor::White, width - clock->Length);
	writeText(clock, ConsoleColor::Gray, clock->Length);
	screen_->WriteLine();
	writeText("  " + gcnew String(L'=', width - 4), ConsoleColor::DarkGray, width);
	screen_->WriteLine();

	// Threads: green = heartbeat OK, red = stopped, grey = still starting.
	writeText("  Threads", ConsoleColor::Gray, 16);
	array<String^>^ names = { "TMM", "LiDAR", "GNSS", "Controller", "VC", "Display", "CrashAvoid" };
	array<unsigned char>^ bits = { bit_TM, bit_LASER, bit_GPS, bit_CONTROLLER, bit_VC, bit_DISPLAY, bit_CRASHAVOIDANCE };
	int used = 16;
	for (int i = 0; i < names->Length; i++) {
		ConsoleColor c = !running_ ? ConsoleColor::DarkGray
			: ((aliveMask_ & bits[i]) ? ConsoleColor::Green : ConsoleColor::Red);
		String^ item = names[i] + "  ";
		if (used + item->Length > width) break;
		writeText(item, c, item->Length);
		used += item->Length;
	}
	writeText("", ConsoleColor::Gray, width - used);
	screen_->WriteLine();
	writeText(rule, ConsoleColor::DarkGray, width);
	screen_->WriteLine();

	writeRow("Controller", "controller", width);
	writeRow("Driver asks", "asked", width);
	writeRow("Sent to UGV", "sent", width);
	writeRow("Crash avoid", "crash", width);
	writeText(rule, ConsoleColor::DarkGray, width);
	screen_->WriteLine();
	writeRow("LiDAR", "lidar", width);
	writeRow("GNSS", "gnss", width);
	writeRow("GNSS CRC", "gnsscrc", width);
	writeText(rule, ConsoleColor::DarkGray, width);
	screen_->WriteLine();

	// Recent events fill the rest of the window, leaving room for the help line.
	writeText("  Recent events", ConsoleColor::Gray, width);
	screen_->WriteLine();
	int rowsUsed = 14;                                           // Rows above the events
	int eventRows = Math::Max(3, Math::Min(12, height - rowsUsed - 3));   // Keep the last window row free (no scrolling)
	int first = Math::Max(0, events_->Count - eventRows);
	for (int i = 0; i < eventRows; i++) {
		int index = first + i;
		if (index < events_->Count) {
			String^ e = events_[index];
			ConsoleColor c = ConsoleColor::Gray;
			if (e->Contains("CRITICAL") || e->Contains("error") || e->Contains("MISMATCH") || e->Contains("could not") || e->Contains("DISCONNECTED")) {
				c = ConsoleColor::Red;
			}
			else if (e->Contains("WARNING") || e->Contains("restarting")) {
				c = ConsoleColor::DarkYellow;
			}
			writeText("    " + e, c, width);
		}
		else {
			writeText("", ConsoleColor::Gray, width);
		}
		screen_->WriteLine();
	}
	writeText(rule, ConsoleColor::DarkGray, width);
	screen_->WriteLine();
	writeText("  RT/R2 forward   LT/L2 reverse   right stick steer   B/Circle or q: shut down", ConsoleColor::DarkCyan, width);
	screen_->WriteLine();
	lastHeight_ = rowsUsed + eventRows + 2;
}
