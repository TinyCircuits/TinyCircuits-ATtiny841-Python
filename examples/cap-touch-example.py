# Capacitive Touch Slider Sensor Wireling Example
# This example will 
# Written by: Laverena Wienclaw for TinyCircuits

import tinycircuits_wireling
import time
import tinycircuits_attiny841

wireling = tinycircuits_wireling.Wireling()
wireling.selectPort(0)

capTouch = tinycircuits_attiny841.ATtiny841()

while True:
    capTouch.