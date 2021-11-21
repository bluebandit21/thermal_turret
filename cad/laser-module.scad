/*
Created By Andrew Kent - SantaBunny
Built on - 11/21/2021

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

Laser_OD = 0.362; // inches
Laser_ID = 0.320; // inches
Laser_wireside_ID = 0.295; // inches
Laser_H = 0.160; // inches

/////////////////////////////////////////

Radius_Fragments = 200;
MM_scalar = 25.4;

spacer = 0.4;

Laser_ODM = Laser_OD * MM_scalar;
Laser_wireside_IDM = Laser_wireside_ID * MM_scalar;
Laser_HM = Laser_H * MM_scalar;

box_size = 25.4; // 12 min // 25.4 - big enough for camera PBC
laser_box = 12;
//////////////////////

box_thickness = 24; // 6

// Use module like this
Laser_Module(box_size, laser_box);

module Laser_Module(box_size, laser_box) {
//main body
	difference() {
		cube($fn = Radius_Fragments, [box_size, laser_box, box_thickness],0);
		translate([box_size / 2,laser_box / 2,box_thickness - 6.1]) {
			translate([0,0,2.1])
				cylinder($fn = Radius_Fragments,Laser_HM, Laser_ODM / 2 + spacer, Laser_ODM / 2 + spacer, 0);
			cylinder($fn = Radius_Fragments, Laser_HM, Laser_wireside_IDM / 2 + spacer, Laser_wireside_IDM / 2 + spacer, 0);
			translate([0,0,-box_thickness + 6])
				cylinder($fn = Radius_Fragments,box_thickness - Laser_HM-1, Laser_ODM / 2 + spacer, Laser_ODM / 2 + spacer, 0);
		};
	};
};


