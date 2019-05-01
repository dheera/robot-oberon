difference() {
    minkowski() {
    cube_center([215,215,30]);
    cylinder(d=15,h=0.0001,$fn=128);
    }
    cube_center([220,220,40]);
}



module cube_center(dims) {
    translate([-dims[0]/2, -dims[1]/2, 0])
    cube(dims);
}