difference() {
    minkowski() {
    cube_center([215,215,15]);
    cylinder(d=15,h=0.0001,$fn=64);
    }
    translate([0,0,2])
    cube_center([220,220,20]);
}



module cube_center(dims) {
    translate([-dims[0]/2, -dims[1]/2, 0])
    cube(dims);
}