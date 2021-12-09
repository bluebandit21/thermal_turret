module stud() {
  difference() {
    cylinder(r=2.2, h=7, $fn=120);
    translate([0,0,3.1]) cylinder(r=.8, h=4, $fn=120);
  }
}

module case() {
  union() {
    difference() {
      cube([90, 2, 58]);
      translate([(90-71.5) / 2,-.1,58-34.5]) cube([71.5, 2.2, 24.5]);
      translate([10, -.1, 10]) rotate([-90,0,0]) cylinder(r=4.1, h=2.2, $fn=120);
      translate([80, -.1, 10]) rotate([-90,0,0]) cylinder(r=3.5, h=2.2, $fn=120);
    }
    translate([(90-71.5)/2-2, 0, 58-34.5-2.3]) rotate([-90,0,0]) stud();
    translate([(90-71.5)/2+71.5+2, 0, 58-34.5-2.3]) rotate([-90,0,0]) stud();
    translate([(90-71.5)/2-2, 0, 58-10+4.2]) rotate([-90,0,0]) stud();
    translate([(90-71.5)/2+71.5+2, 0, 58-10+4.2]) rotate([-90,0,0]) stud();
    
    difference() {
      cube([90, 20, 2]);  // bottom wall
        translate([10,17,-1]) cylinder(r=.5, h=5, $fn=120);   // screw holes
        translate([80, 17, -1]) cylinder(r=.5, h=5, $fn=120); 
    }
    
    difference() {
      translate([0,0,56]) cube([90, 20, 2]); // top wall
      translate([10,17,55]) cylinder(r=.5, h=5, $fn=120);   // screw holes
      translate([80, 17, 55]) cylinder(r=.5, h=5, $fn=120); 
    }
      
    
    difference() {
      translate([88,0,0]) cube([2, 20, 58]);  // left wall
      translate([87.9,20-4,58-17]) cube([2.2, 4.1, 10]); // USB port
    }

    difference() {    
        cube([2, 20, 58]); // right wall
        translate([-.5, 13.5,6]) rotate([90,0,90]) switchplate();
    } 
  }
}

module switchplate() {
  cylinder(r=1, h=3, $fn=120);
  translate([0, 14.75, 0]) cylinder(r=1, h=3, $fn=120);
  translate([-4.5/2, 14.75/2 - 7.25/2, 0]) cube([4.5, 7.25, 3]);
}

rotate([90,0,0]) case();

