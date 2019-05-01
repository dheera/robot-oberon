difference() {
    cube_center([220,110,2]);
    for(i = [-100:10:100]){
        for(j = [-45:10:45]){
            if(!((i==100 || i==-100) && j==45)) {
                translate([i,j,0])
                cylinder(d=1.7,h=3,$fn=16);
            }
        }
    }
    for(i=[-90:20:90]) {
        translate([i,50,0])
        cylinder(d1=3.5,d2=6,h=2,$fn=16);
    }
    for(j=[-45:20:35]) {
        translate([-105,j,0])
        cylinder(d=3.5,d2=6,h=2,$fn=16);
    }
    for(j=[-45:20:35]) {
        translate([105,j,0])
        cylinder(d1=3.5,d2=6,h=2,$fn=16);
    }
    
    translate([-105,50,0])
    cube_center([10,10,3]);
    
    translate([105,50,0])
    cube_center([10,10,3]);
    
    translate([50,35,0])
    cube_center([12,12,3]);
    
    translate([-50,35,0])
    cube_center([12,12,3]);
    
    translate([90,-5,0])
    cube_center([12,12,3]);
    
    translate([-90,-5,0])
    cube_center([12,12,3]);
    
    translate([0,-5,0])
    cube_center([12,12,3]);
    
    translate([0,-55,0])
    cylinder(d=30,h=3);
}


module cube_center(dims) {
    translate([-dims[0]/2, -dims[1]/2, 0])
    cube(dims);
}