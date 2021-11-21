/*
Created By Andrew Kent - SantaBunny
Built on - 11/21/2021

v1 - 11/21/2021



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

Camera_Board_W = 0.982; // inches -- top to bottom
Camera_Board_L = 0.950; // inches <-- has 0.630 ribbon coming out of bottom center -- left to right

Camera_D = 0.550; // inches


//////////////////////////////////////////////
Radius_Fragments = 200;
MM_scalar = 25.4;

spacer = 0.4;
box_thickness = 24;

Camera_Board_LM = Camera_Board_L * MM_scalar;
Camera_Board_WM = Camera_Board_W * MM_scalar;
Camera_DM = Camera_D * MM_scalar;
Ribbon_M = 16.002;


screw_center_M = 0.325 * MM_scalar;
support_RM = 0.12 * MM_scalar;
support_depth_M = 5;

/////////////////////////////////////////////

Camera_Module();

module Camera_Module() {
//main body
	union() {
		difference() {
			cube($fn = Radius_Fragments, [Camera_Board_LM+6, Camera_Board_WM+6, box_thickness],0);
			// Camera Board
			translate([2.8,2.8,box_thickness-Camera_DM -1])
				cube($fn = Radius_Fragments, [Camera_Board_LM+spacer, Camera_Board_WM+spacer, box_thickness - Camera_DM +support_depth_M], 0);
			// Ribbon Cable
			translate([(Camera_Board_LM+6 - Ribbon_M) / 2,-0.1,box_thickness - Camera_DM + 3])
				cube($fn = Radius_Fragments, [Ribbon_M+spacer, Camera_Board_WM, box_thickness - Camera_DM + 1], 0);
		};
		translate([5,Camera_Board_WM + 1,box_thickness - Camera_DM - 2])
		rotate([0,0,90])
			Corner();
		translate([Camera_Board_LM + 1,Camera_Board_WM + 1,box_thickness - Camera_DM - 2])
		rotate([0,0,0])
			Corner();
		translate([5,5,box_thickness - Camera_DM - 2])
		rotate([0,0,180])
			Corner();
		translate([Camera_Board_LM + 1,5,box_thickness - Camera_DM - 2])
		rotate([0,0,-90])
			Corner();
		//cylinder($fn = Radius_Fragments, support_depth_M, support_RM, support_RM, 0);
	};
};

module Corner() {
	union() {
		translate([0,0,0])
			cube($fn = Radius_Fragments, [support_RM,support_RM,support_depth_M], 0);
		cylinder($fn = Radius_Fragments, support_depth_M, support_RM, support_RM, 0);
	}
}

