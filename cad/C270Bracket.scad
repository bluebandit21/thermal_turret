$fn=90;
width=7;
height=7;

lampWidth=11;

smallScrew=2.8;
largeScrew=3.8;

mThreeHead=6;

lampScrew=5;

module lamp(){
	translate([40,0,0])
	difference() {
	cube([lampWidth, 40, lampWidth]);
	rotate([90,0,0])
		translate([lampWidth/2, lampWidth/2,-5])
		cylinder(r=smallScrew/2, h=10+1, center=true);

	rotate([0, 90, 0])
		translate([-lampWidth/2, 40-lampScrew, lampWidth/2])
		cylinder(r=lampScrew/2, h=lampWidth*2, center=true);   
    }
}

module bracket() {
	difference() {

	translate([0, -width/2, 0])  // Goofed up my math in there somewhere
	hull() {
		rotate([0,90,0])
		translate([-width/2,width,0])
		cylinder(r=width/2, h=width*3);
		translate([width*2/3, width*3, 0])
		cube([lampWidth, 1, width]);
	}
  
	translate([width, -1, -1])
		cube([width, width+1, width+3]);

	rotate([0,90,0])
		translate([-width/2, width/2, 0])
		cylinder(r=largeScrew/2, h=width);
    
		rotate([0,90,0])
		translate([-width/2, width/2, width*2])
		cylinder(r=smallScrew/2, h=width);

		rotate([90,0,0])
		translate([width*3/2, width/2, -width*3])
		cylinder(r=largeScrew/2, h=width*3);

      
		rotate([90,0,0])
			translate([width*3/2, width/2, -width-2])
			cylinder(r=mThreeHead/2, h=width);      
	}
}
