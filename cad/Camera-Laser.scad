/*
Created By Andrew Kent - SantaBunny
Built on - 11/16/2021

v1 - 11/16/2021



					  _.-''''''-._
				  _.-'			  '-.
				.'				  _  '-
			   /				-' '-_ '_
			  /			        \     '8888     
			 /					 \    888888
			/					  \    8888
		   /					   \	
		  /						    \
		 /						     \  
		/						      \  
   ____/__  ______  ______  ______  ___\___
  /       \/      \/      \/      \/       \
 /										    \
|										     |
 \										    /
  \										   /
   \______/\______/\______/\______/\______/


Post-Fixes
------------------
D = diameter
I = Inner
L = Length
M = Milimeters
O = Outer
R = Radius
W = Width


*/
use <laser-module.scad>
use <camera-module.scad>

////////////////////////////////////////////////////////////////////////////
// Everything in this secion is inches!!!! DO NOT USE BASE NUMBERS		  //
////////////////////////////////////////////////////////////////////////////
Laser_OD = 0.362; // inches
Laser_ID = 0.320; // inches
Laser_wireside_ID = 0.295; // inches
Laser_H = 0.160; // inches


Mount_Box_W = 1.190; //inches -- Max space 1.195
Mount_Box_L = 1.185; // inches -- hook top 1.06 -- Diff = 0.125
Mount_Box_H = 0.395; // inches

Camera_Board_W = 0.982; // inches
Camera_Board_L = 0.950; // inches <-- has 0.630 ribbon coming out of bottom center

Camera_D = 0.550; // inches



////////////////////////////////////////////////////////////////////////////
// Everything above this secion is inches!!!! DO NOT USE BASE NUMBERS	  //
////////////////////////////////////////////////////////////////////////////

Radius_Fragments = 200;
MM_scalar = 25.4;

Mount_Box_LM = Mount_Box_L * MM_scalar;
Mount_Box_WM = Mount_Box_W * MM_scalar;
Mount_Box_HM = Mount_Box_H * MM_scalar;

Camera_Board_LM = Camera_Board_L * MM_scalar;
Camera_Board_WM = Camera_Board_W * MM_scalar;


// Mounting Base
cube($fn = Radius_Fragments, [Mount_Box_WM, Mount_Box_LM, Mount_Box_HM], 0);

//spacer box
translate ([Mount_Box_WM / 4,Mount_Box_LM / 4,Mount_Box_HM])
	cube($fn = Radius_Fragments, [Mount_Box_WM / 2, Mount_Box_LM / 2, Mount_Box_HM - 4], 0);

translate([Mount_Box_WM / 2 + (Camera_Board_LM + 12) /4,Mount_Box_LM / 2 + 12,Mount_Box_HM * 2 - 4 + Camera_Board_LM + 10])
rotate([90,90,0])
	Laser_Module(Camera_Board_LM + 10, 12);


translate([Mount_Box_WM / 2 - (Camera_Board_WM + 6) + (Camera_Board_LM + 12) / 4,Mount_Box_LM / 2 + 12,Mount_Box_HM * 2])
rotate ([90,0,0])
	Camera_Module();