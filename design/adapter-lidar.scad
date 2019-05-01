difference() {
    cylinder(d=76,h=10,$fn=256);
    
    translate([-41.5/2,-41.5/2,0]) {
    cylinder(d2=3.2,d1=6.9,h=1.7,$fn=32);
    cylinder(d=3.2,h=12,$fn=32);
    }
    translate([-41.5/2,41.5/2,0]) {
    cylinder(d2=3.2,d1=6.9,h=1.7,$fn=32);
    cylinder(d=3.2,h=12,$fn=32);
    }
    translate([41.5/2,-41.5/2,0]) {
    cylinder(d21=3.2,d1=6.9,h=1.7,$fn=32);
    cylinder(d=3.2,h=12,$fn=32);
    }
    translate([41.5/2,41.5/2,0]) {
    cylinder(d2=3.2,d1=6.9,h=1.7,$fn=32);
    cylinder(d=3.2,h=12,$fn=32);
    }
    
    translate([0,0,2])
    cylinder(d=40,h=15,$fn=256);
    
    translate([0,0,2])
    translate([-20.5/2,-50,0])
    cube([20.5,100,9]);
    
    translate([0,-20,0]) {
    translate([0,0,1])
    translate([-20.5/2,-3/2,0])
    cube([20.5,3,9]);
    
    translate([0,34,1])
    translate([-20.5/2,-11/2,0])
    cube([20.5,11,9]);
    
    translate([-14/2,10,0])
    cylinder(d=1.8,h=3,$fn=16);
    translate([14/2,10,0])
    cylinder(d=1.8,h=3,$fn=16);
    }
}


difference() {
minkowski() {
difference() {
translate([-50/2,-80/2,0])
cube([50,80,2]);
translate([-49.99/2,-80/2,0])
cube([49.99,80,2]);
}
    
cylinder(d=10,h=0.0001,$fn=32);
}

translate([-25,-40,0])
cylinder(d=2,h=10, $fn=32);

translate([-25,40,0])
cylinder(d=2,h=10, $fn=32);

translate([25,-40,0])
cylinder(d=2,h=10, $fn=32);

translate([25,40,0])
cylinder(d=2,h=10, $fn=32);

    translate([-41.5/2,-41.5/2,0]) {
    cylinder(d2=3.2,d1=6.9,h=1.7,$fn=32);
    cylinder(d=3.2,h=12,$fn=32);
    }
    translate([-41.5/2,41.5/2,0]) {
    cylinder(d2=3.2,d1=6.9,h=1.7,$fn=32);
    cylinder(d=3.2,h=12,$fn=32);
    }
    translate([41.5/2,-41.5/2,0]) {
    cylinder(d21=3.2,d1=6.9,h=1.7,$fn=32);
    cylinder(d=3.2,h=12,$fn=32);
    }
    translate([41.5/2,41.5/2,0]) {
    cylinder(d2=3.2,d1=6.9,h=1.7,$fn=32);
    cylinder(d=3.2,h=12,$fn=32);
    }

}

