# --- R24D boot init (safe) ---
tasmota.cmd("SerialLog 0")
tasmota.cmd("Baudrate 115200")

# Load the driver once at boot
load("r24d.bec")

# mmWave settings
tasmota.cmd("setScene 3") ;
tasmota.cmd("TelePeriod 30");
tasmota.cmd("SetSensitivity 1");
tasmota.cmd("SetDelay 2")
