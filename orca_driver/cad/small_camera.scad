// in mm

// Generic tube, radius is outer radius
module tube(height, radius, wall) {
  difference() {
    cylinder(h=height, r=radius);
    translate([0, 0, -1]) cylinder(h=height+2, r=radius-wall);
  }
}

// Blue Robotics 2" series flange
module flange() {
  // Part outside of acrylic tube
  color("gray") tube(6, 57/2, 7/2+12/2);
  
  translate([0, 0, 6]) color("gray") difference() {
    
    // Part inside acrylic tube
    tube(21.5-6, 50/2, 12/2);
    
    // Screw holes TODO too long
    translate([21.5, 0, 0]) cylinder(h=19, r=1.5);
    translate([-21.5, 0, 0]) cylinder(h=19, r=1.5);
    translate([0, 21.5, 0]) cylinder(h=19, r=1.5);
    translate([0, -21.5, 0]) cylinder(h=19, r=1.5);
  }
}

// Blue Robotics camera w/ wide angle lens 
module camera() {
  color("blue") union() {
    // Connector
    translate([-25/2+2.5, -10, 22.3]) cube([20, 5, 2.5]);
    
    translate([-25/2, -10, 22.3]) difference() {
      // Board
      cube([25, 24.5, 1]);
      
      // Screw holes
      translate([2, 8.8, -1]) cylinder(h=3, r=1);
      translate([2, 8.8+12.5, -1]) cylinder(h=3, r=1);
      translate([2+21, 8.8, -1]) cylinder(h=3, r=1);
      translate([2+21, 8.8+12.5, -1]) cylinder(h=3, r=1);
    }
    
    // Lens
    cylinder(h=22.3, r=8.3);
  }
}

// Pi Zero
module pizero() {
  union() {
    // Camera connector
    translate([15/2, 1, 0]) cube([15, 2.5, 5]);

    // USB connector
    translate([30-4, 1, 22]) cube([5, 2, 8]);
    
    difference() {
      // Board
      cube([30, 1, 65]);
      
      // Screw holes
      translate([3.5, 1.5, 3.5]) rotate([90, 0, 0]) cylinder(h=3, r=1);
      translate([30-3.5, 1.5, 3.5]) rotate([90, 0, 0]) cylinder(h=3, r=1);
      translate([3.5, 1.5, 65-3.5]) rotate([90, 0, 0]) cylinder(h=3, r=1);
      translate([30-3.5, 1.5, 65-3.5]) rotate([90, 0, 0]) cylinder(h=3, r=1);
    }
  }
}

// Camera bracket
module camerabracket() {
  color("green") difference() {
    // Disk
    cylinder(h=2, r=50/2);

    // Cutout for camera cable
    translate([-10, -15, -1]) cube([20, 2, 4]);

    // Screw holes for flange
    translate([21.5, 0, -1]) cylinder(4, r=1.5);
    translate([-21.5, 0, -1]) cylinder(4, r=1.5);
    translate([0, 21.5, -1]) cylinder(4, r=1.5);
    translate([0, -21.5, -1]) cylinder(4, r=1.5);
   
    // Screw holes for camera mount
    translate([-21/2, 0, -1]) cylinder(4, r=1);
    translate([-21/2, 0+12.5, -1]) cylinder(4, r=1);
    translate([21/2, 0, -1]) cylinder(4, r=1);
    translate([21/2, 0+12.5, -1]) cylinder(4, r=1);
  }
}

// Pi bracket
module pibracket() {
  color("green") difference() {
    // Disk
    cylinder(h=2, r=50/2);
    
    // Cutout for USB cable
    translate([0, 0, -1]) cylinder(h=4, r=4);

    // Screw holes for flange
    translate([21.5, 0, -1]) cylinder(h=4, r=1.5);
    translate([-21.5, 0, -1]) cylinder(h=4, r=1.5);
    translate([0, 21.5, -1]) cylinder(h=4, r=1.5);
    translate([0, -21.5, -1]) cylinder(h=4, r=1.5);

    // Screw holes for the Pi
    translate([3.5, 3, 3.5]) rotate([90, 0, 0]) cylinder(h=4, r=1);
    translate([30-3.5, 3, 3.5]) rotate([90, 0, 0]) cylinder(h=4, r=1);
    translate([3.5, 3, 65-3.5]) rotate([90, 0, 0]) cylinder(h=4, r=1);
    translate([30-3.5, 3, 65-3.5]) rotate([90, 0, 0]) cylinder(h=4, r=1);
  }
}

// Top (camera side) flange
flange();

// Camera
// Assume sphere radius = 20mm, center of sphere is 2mm into flange,
// and lens center of projection is 15mm from bottom of circuit board
// This places top of lens 5mm above flange
translate([0, 0, -5]) camera();

// Camera bracket
translate([0, 0, 21.5]) camerabracket();

// Pi Zero
translate([-24, 4, 35]) pizero();

// Pi bracket
translate([0, 0, 132-21.5-2]) pibracket();

// Bottom (wire side) flange
translate([0, 0, 132]) rotate([0, 180, 0]) flange();

// Tube
color("white", 0.1) translate([0, 0, 6]) tube(120, 57.2/2, 3.2);