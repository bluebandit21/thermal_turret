/*
Created By Andrew Kent - SantaBunny
Built on - 11/16/2021

v1 - 11/16/2021
v2 - 12/07/2021



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
use <C270Bracket.scad>

////////////////////////////////////////////////////////////////////////////
// Everything in this secion is inches!!!! DO NOT USE BASE NUMBERS		  //
////////////////////////////////////////////////////////////////////////////
Laser_OD = 0.362; // inches
Laser_ID = 0.320; // inches
Laser_wireside_ID = 0.295; // inches
Laser_H = 0.160; // inches


Mount_Box_W = 1.185; //inches -- Max space 1.195
Mount_Box_L = 1.185; // inches -- hook top 1.06 -- Diff = 0.125
Mount_Box_H = 0.3937; // inches

// Rpi Cam
Camera_Board_W = 0.982; // inches
Camera_Board_L = 0.950; // inches <-- has 0.630 ribbon coming out of bottom center
Camera_D = 0.550; // inches

// Logitech C310
Camera_H = 1 + (7/32);
Camera_Front2Back = 0.75;

////////////////////////////////////////////////////////////////////////////
// Everything above this secion is inches!!!! DO NOT USE BASE NUMBERS	  //
////////////////////////////////////////////////////////////////////////////
spacer = 0.4;
Radius_Fragments = 200;
MM_scalar = 25.4;

Mount_Box_LM = Mount_Box_L * MM_scalar;
Mount_Box_WM = Mount_Box_W * MM_scalar;
Mount_Box_HM = Mount_Box_H * MM_scalar;
pyramid_Sidelength_M = sqrt(2 * pow(Mount_Box_LM, 2) );



// Rpi Cam
Camera_Board_LM = Camera_Board_L * MM_scalar;
Camera_Board_WM = Camera_Board_W * MM_scalar;


// Logitech C310
Camera_Front2Back_M = Camera_Front2Back * MM_scalar;
Camera_HM = Camera_H * MM_scalar;

// Laser
Laser_ODM = Laser_OD * MM_scalar;
Laser_wireside_IDM = Laser_wireside_ID * MM_scalar;
Laser_HM = Laser_H * MM_scalar;
box_size = Camera_Board_LM + 10; // 12 min // 25.4 - big enough for camera PBC
laser_box = 12;
box_thickness = 24; // 6
back_thickenss = box_thickness - Laser_HM - 4;

/////////////////////////////////////////////////////////////////////////////////////


// Mounting Base
translate([0,Mount_Box_LM * 3 / 2,Mount_Box_HM + 8])
cube($fn = Radius_Fragments, [Mount_Box_WM, Mount_Box_HM, Mount_Box_LM], 0);

// Base support cube
//translate([5,Mount_Box_LM * 5/4,Mount_Box_HM + 8])
//cube($fn = Radius_Fragments, [Mount_Box_WM - 10, Mount_Box_HM, Mount_Box_LM], 0);]\

// Pyramid support
translate([Mount_Box_LM/2,Mount_Box_LM * 5/4 + Mount_Box_LM/4,Mount_Box_HM + 8 + Mount_Box_LM/2])
rotate([90,-45,0])
cylinder($fn = 4, Mount_Box_LM/4, (pyramid_Sidelength_M - 10)/2, pyramid_Sidelength_M/2, 0);


// cube for cut bottom
translate([0,Mount_Box_LM * 5/4,Mount_Box_HM + Mount_Box_LM + 3 ])
cube($fn = Radius_Fragments, [Mount_Box_WM, Mount_Box_HM, 5], 0);
// cube for cut top
translate([0,Mount_Box_LM * 5/4,Mount_Box_HM + 8])
cube($fn = Radius_Fragments, [Mount_Box_WM, Mount_Box_HM, 5], 0);





//cube($fn = Radius_Fragments, [Mount_Box_WM - 10, Mount_Box_HM, Mount_Box_LM], 0);

//translate ([Mount_Box_WM / 4,Mount_Box_LM / 4,Mount_Box_HM])
//	cube($fn = Radius_Fragments, [Mount_Box_WM / 2, Mount_Box_LM, Mount_Box_HM - 4], 0);



// back support
	// Main up Block
difference() {
	union() {
	translate ([0,Mount_Box_LM * 5 / 4 - 11.57,Mount_Box_HM + 5])
		cube($fn = Radius_Fragments, [Mount_Box_WM,11.57,44],0);
		
	}
	
	// Hole for Laser wires
	translate([-((Camera_Board_LM + 10) - Mount_Box_WM) / 2,Mount_Box_LM / 2 + 12,Mount_Box_HM * 2 - 4 + Camera_HM])
	rotate([90,0,0]){	
		translate([box_size / 2,laser_box / 2,box_thickness - 6.1]) {
			translate([0,0,-box_thickness -6])
			cylinder($fn = Radius_Fragments, back_thickenss + 12, Laser_ODM / 2 + spacer, Laser_ODM / 2 + spacer, 0);
		}
	}
	
	// Top angle aesthetic cut
	translate ([-0.1,Mount_Box_LM * 5 / 4 - 13.57,Mount_Box_HM * 2 + 10 + Camera_HM])
	rotate([-33.777,0,0])
		cube($fn = Radius_Fragments, [Mount_Box_WM + 2, 20, 20], 0);
		
	// cylinder cut for camera
	translate([bracket_centered_M  + camera_offset_M + 7,      Camera_Front2Back_M + 7,         Mount_Box_HM * 2 - 4+ (Camera_HM / 2)])
	rotate([0,90,0])
	cylinder($fn = Radius_Fragments, r=7/2, h=7);
}
/*
	// Back angle
translate([0,Mount_Box_LM,5])
	rotate([33.777,0,0])
		cube($fn = Radius_Fragments, [Mount_Box_WM,9,5], 0);

	//Back Support center angle
translate([1.5875,Mount_Box_LM - 10.0375,Mount_Box_HM - 4])
rotate([33.777,0,0])
	cube($fn = Radius_Fragments, [Mount_Box_LM - 3.175,12,4], 0);
*/


// Laser
translate([-((Camera_Board_LM + 6) - Mount_Box_WM) / 2,Mount_Box_LM / 2 + 12,Mount_Box_HM * 2 - 4 + Camera_HM])
rotate([90,0,0])
	Laser_Module(Camera_Board_LM + 6, 12, 24);

// Camera
// translate([Mount_Box_WM / 2 - (Camera_Board_WM + 6) + (Camera_Board_LM + 12) / 4,Mount_Box_LM / 2 + 12,Mount_Box_HM * 2])
// rotate ([90,0,0])
// Camera_Module();

bracket_centered_M = ((Mount_Box_LM) - 21) / 2;
camera_offset_M = 13/32 * MM_scalar;

difference() {
	translate([bracket_centered_M  + camera_offset_M,   Camera_Front2Back_M,   Mount_Box_HM * 2 - 4+ (Camera_HM / 2)  - 7/2])
	bracket();
	
	// cylinder cut for camera
	translate([bracket_centered_M  + camera_offset_M + 7,      Camera_Front2Back_M + 7,         Mount_Box_HM * 2 - 4+ (Camera_HM / 2)])
	rotate([0,90,0])
	cylinder($fn = Radius_Fragments, r=7/2, h=7);
}




