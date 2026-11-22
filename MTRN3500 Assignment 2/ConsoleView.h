#pragma once
/*
ConsoleView.h - live status screen for the console (not one of the assigned modules; display helper only).

When the program runs in a console window, the screen is redrawn in place 4 times a second:
thread status, driving, crash avoidance, LiDAR, GNSS, and the most recent event messages.
Modules put their latest values in named fields with set(). Anything written with Console::WriteLine
(connections, errors, shutdown) is caught and shown under "Recent events".

When the output is redirected (to a file or a test script), the view stays off and modules print
plain lines instead, so logs and tests still work.
*/
#using <System.dll>

using namespace System;
using namespace System::Threading;
using namespace System::Collections::Generic;

ref class ConsoleView abstract sealed {
public:
    static void start();                                          // Called once by TMM at start-up
    static void stop();                                           // Back to plain output (TMM, at shutdown)
    static bool isActive();
    static void set(String^ field, String^ text, ConsoleColor colour);
    static void setThreads(unsigned char aliveMask, bool running);  // bit_ flags from SMObjects.h
    static void addChar(wchar_t c);                               // Used by the output catcher

private:
    static void onTimer(Object^ unused);
    static void render();
    static void writeRow(String^ label, String^ field, int width);
    static void writeText(String^ text, ConsoleColor colour, int width);

    static Object^ lock_ = gcnew Object();
    static bool active_ = false;
    static IO::TextWriter^ screen_ = nullptr;                    // The real console output
    static Timer^ timer_ = nullptr;
    static Diagnostics::Stopwatch^ upTime_ = nullptr;
    static Dictionary<String^, String^>^ text_ = gcnew Dictionary<String^, String^>();
    static Dictionary<String^, ConsoleColor>^ colour_ = gcnew Dictionary<String^, ConsoleColor>();
    static List<String^>^ events_ = gcnew List<String^>();
    static Text::StringBuilder^ partLine_ = gcnew Text::StringBuilder();
    static unsigned char aliveMask_ = 0;
    static bool running_ = false;
    static int lastHeight_ = 0;

    literal int REDRAW_PERIOD_MS = 250;
    literal int MAX_EVENTS = 200;
    literal int MAX_WIDTH = 100;
};

// Catches Console::Write/WriteLine while the view is active and passes the text to ConsoleView.
ref class EventCatcher : public IO::TextWriter {
public:
    property Text::Encoding^ Encoding {
        virtual Text::Encoding^ get() override { return Text::Encoding::UTF8; }
    }
    virtual void Write(wchar_t c) override { ConsoleView::addChar(c); }
    virtual void Write(String^ s) override {
        if (s == nullptr) return;
        for each (wchar_t c in s) ConsoleView::addChar(c);
    }
    virtual void WriteLine(String^ s) override { Write(s); ConsoleView::addChar(L'\n'); }
    virtual void WriteLine() override { ConsoleView::addChar(L'\n'); }
};
