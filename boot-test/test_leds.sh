#!/bin/bash
# Store first parameter in a variable, which should be the log file location.
LOG_FILE="$1"
# Set a default log file location if the parameter was empty, i.e. not specified.
if [ -z "$LOG_FILE" ]
then
  LOG_FILE="/var/log/testlog.txt"
fi

echo "Blinking LEDS 1-4. Setting LED_1v8 to HIGH" >> "$LOG_FILE"
python3 /home/trident/boot-test/blink.py