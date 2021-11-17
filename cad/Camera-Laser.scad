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


// Mounting Base
cube($fn = Radius_Fragments, [Mount_Box_WM, Mount_Box_LM, Mount_Box_HM], 0);



