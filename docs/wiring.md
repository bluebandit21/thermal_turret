
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

TODO: // describe op-amp circuit coming off of data pin