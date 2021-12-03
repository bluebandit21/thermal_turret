
### **Gimbal:**  
Servomotors red wires need to be connected to +5 V, brown wires need to be connected to ground.  
The orange wire of the pitch servo needs to be connected to pin A9 (labelled "D8" on board surface)  
The orange wire of the yaw servo needs to be connected to pin A8 (labelled "D7" on board surface)  


### **Infrared thermometer:**  
Red wire needs to be connected to +3.3 V (not 5V!!!!!!!!!!!!!), black to ground, with a 0.1 uF decoupling capacitor between the two  
Green (SCL) needs to be connected to pin B6 (ninth pin down on the middle row of pins on the right (oriented with ST logo on bottom))  
Orange (SDA) needs to be connected to pin B7 (ninth pin from the bottom on the left row of pins on the left (oriented with ST logo on bottom))  

### **Contact thermometer:**  
With flat side of device oriented up and bottom of device (pin side) pointed towards sky,  
Left leg connected to voltage supply (3.3v or 5v both fine, operating range is 2.7 - 5.5 volts)
Right leg connected to ground
0.1 uF capacitor (labelled "BC 104") between high and low pins

Output pin of op-amp circuit connected to pin A1 (fifth pin from the bottom on the middle row of pins on the left (oriented with ST logo on bottom))

TODO: // describe op-amp circuit coming off of data pin


### **Vision interface:**
Target left wire connected to pin PB3  (Fourth pin from bottom on middle row of pins on the right (oriented with ST logo on bottom)), other end connected to Jetson Nano pin //TODO: ADD ME

Target right wire connected to pin PB4 (Sixth pin from bottom on middle row of pins on the right (oriented with ST logo on bottom)), other end connected to Jetson Nano pin //TODO: ADD ME

Target up wire connected to pin PB5    (Fifth pin from bottom on middle row of pins on the right (oriented with ST logo on bottom)), other end connected to Jetson Nano pin //TODO: ADD ME

Target down wire connected to pin PB10 (Seventh pin from bottom on middle row of pins on the right (oriented with ST logo on bottom)), other end connected to Jetson Nano pin //TODO: ADD ME


### **Logic Analyzer:**
Yellow wire connected to CH2 is SDA, grey wire connected to CH3 is SCL
(If these are set up properly, logic analyzer very nicely properly logs output for us)