L1=86;
W1=58;

difference() {
minkowski() {
translate([-L1/2,-W1/2])
cube([L1,W1,2]);
cylinder(d=9,h=0.0001,$fn=32);

}


for(i=[-4.5:4.5]) {
    for(j=[-3:3]) {
      
      if(!(abs(i)==4 && abs(j)==3)) {
      translate([i*10,j*10,0])
      cylinder(d=2,h=4,$fn=16);
      }
        
    }}
    
    minkowski() {
        translate([-L1/2+7,-W1/2+7])
cube([L1-14,W1-14,2]);
cylinder(d=6,h=0.0001,$fn=32);
    }

translate([-L1/2,-W1/2,0])
standoffh();
translate([L1/2,-W1/2,0])
standoffh();
translate([-L1/2,W1/2,0])
standoffh();
translate([L1/2,W1/2,0])
standoffh();
}

translate([-L1/2,-W1/2,0])
standoff();
translate([L1/2,-W1/2,0])
standoff();
translate([-L1/2,W1/2,0])
standoff();
translate([L1/2,W1/2,0])
standoff();

module standoff() {
    difference() {
        cylinder(d1=9,d2=7,h=4,$fn=64);
        cylinder(d=4.8,h=4,$fn=32);
    }
}
module standoffh() {

        cylinder(d=6,h=4,$fn=32);
}