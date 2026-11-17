"""
Builds a TEST-ONLY copy of the exe in which the Controller reads its inputs from a text file
instead of an Xbox controller, so a test script can "drive" the vehicle. The repo code is not
changed: the project is copied to <out_dir>, one line of Controller.cpp is patched there, and
that copy is built.

The input file (path in the MTRN_TEST_INPUT environment variable) holds one line:
    <connected 0/1> <rightTrigger 0..1> <leftTrigger 0..1> <rightThumbX -1..1> <buttonB 0/1>
Everything after GetState() (trigger/stick mapping, deadzone, sign, shutdown button) is the real code.

Usage: python tools/build_test_exe.py <out_dir>      (prints the test exe path)
"""
import os, shutil, subprocess, sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.abspath(sys.argv[1])
MSBUILD = r"C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe"

ORIGINAL = "\tcontrollerState state = controller_->GetState();"
PATCH = r"""	controllerState state = {};
	try {
		array<String^>^ f = IO::File::ReadAllText(Environment::GetEnvironmentVariable("MTRN_TEST_INPUT"))->Trim()->Split(' ');
		state.isConnected = f[0] == "1";
		state.rightTrigger = Double::Parse(f[1]);
		state.leftTrigger = Double::Parse(f[2]);
		state.rightThumbX = Double::Parse(f[3]);
		state.buttonB = f[4] == "1";
	}
	catch (Exception^) { state.isConnected = false; }"""

if os.path.exists(OUT):
    shutil.rmtree(OUT)
shutil.copytree(REPO, OUT, ignore=shutil.ignore_patterns(".git", ".vs", "x64", "Executables", "tools"))
cpp = os.path.join(OUT, "MTRN3500 Assignment 2", "Controller.cpp")
src = open(cpp, encoding="utf-8").read()
assert src.count(ORIGINAL) == 1, "Controller.cpp no longer has the GetState() line this script patches"
open(cpp, "w", encoding="utf-8").write(src.replace(ORIGINAL, PATCH))

r = subprocess.run([MSBUILD, os.path.join(OUT, "MTRN3500 Assignment 2.sln"), "-p:Configuration=Debug",
                    "-p:Platform=x64", "-p:PlatformToolset=v145", "-m", "-v:m", "-nologo"],
                   capture_output=True, text=True)
if r.returncode != 0:
    print(r.stdout[-3000:])
    sys.exit("test build failed")
print(os.path.join(OUT, "Executables", "MTRN3500 Assignment 2.exe"))
