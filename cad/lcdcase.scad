pcb_x = 116.5;
pcb_y = 42.5;
pcb_z = 5.5;
pcb_bolt_d = 4;  // is marginally too small for M3 :(
pcb_bolt_r = pcb_bolt_d/2;
pcb_bolt_inset = 4;

lcd_x_offset = 6.9;
lcd_y_offset = 6.4;
lcd_view_x_offset = 6.75;
lcd_view_y_offset = 4;
lcd_view_z = 10;
lcd_x = 102.2;
lcd_y = 32.2;
lcd_z = 11.3;

pins_x_offset = 8.5;
pins_y_offset = 0;
pins_x = 43;
pins_y = 5.5;
pins_z = 20;

case_wall = 3.5;
case_d = pcb_z + lcd_z + (2 * case_wall);
case_r  = case_d/2;
case_split = pins_x_offset + (pins_x * .75);

difference()
{
	lcdcase_all();
	translate([-pcb_x*2,0,0])
	translate([case_split,-pcb_y/2,-case_r/2]) cube([pcb_x*2,pcb_y*2,case_r*4]);
}



module lcdcase_all()
{
	difference()
	{
		lcdcase();
		lcdunit();
	}
	pcb_boltholes();
}

module lcdcase()
{
	translate([0,0,case_r-case_wall])
	rotate([0,90,0])
	minkowski()
	{
		sphere(r=case_r,center=true);
		cube([0.01, pcb_y, pcb_x]);
	}
}

module lcdunit()
{
	union() 
	{
		lcd();
		pcb();
		pins();
	}
}

module pins()
{
	translate([pins_x_offset, pins_y_offset, -pins_z + 0.02])
		cube([pins_x, pins_y, pins_z]);
}

module lcd()
{
	translate([lcd_x_offset, lcd_y_offset, pcb_z])
	{
		cube([lcd_x, lcd_y, lcd_z]);
		translate([lcd_view_x_offset, lcd_view_y_offset, lcd_z - 0.02])
			cube([lcd_x - (2*lcd_view_x_offset), lcd_y - (2*lcd_view_y_offset), lcd_view_z]);
	}
}

module pcb()
{
	difference()
	{
		cube([pcb_x, pcb_y, pcb_z]);
		// pcb_boltholes();
	}
}

module pcb_boltholes()
{
	for(y = [pcb_y - pcb_bolt_inset, pcb_bolt_inset])
	{
		translate([pcb_bolt_inset,y,-0.01])
		{
			funky_holegrabber(radius = pcb_bolt_r, height = pcb_z + 0.02);
			// cylinder(r=pcb_bolt_r, h=pcb_z+0.02);
		}
	}

	for(q = [pcb_y - pcb_bolt_inset, pcb_bolt_inset])
	{
		translate([pcb_x - pcb_bolt_inset,q,-0.01])
		{
			rotate([0,0,180]) funky_holegrabber(radius = pcb_bolt_r, height = pcb_z + 0.02);
			// cylinder(r=pcb_bolt_r, h=pcb_z+0.02);
		}
	}

}

module funky_holegrabber(height = 1, radius = 5)
{
	difference()
	{
		translate([0,0,-0.01]) cylinder(r=radius, h = height+0.02);
		translate([0,0,height/2]) cube([radius * 2.001, radius * 2.01, height * .33], center = true);
		translate([radius,0,height/2]) rotate([90,180,0]) cylinder(r = height, h = (radius*2) + 0.023, center=true, $fn = 3);
	}
}

