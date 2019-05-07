L1=22;
W1=15;

difference() {
minkowski() {
translate([-L1/2,-W1/2])
cube([L1,W1,2]);
cylinder(d=9,h=0.0001,$fn=32);

}

translate([10,0,0])
cylinder(d=2,h=4,$fn=16);

translate([-10,0,0])
cylinder(d=2,h=4,$fn=16);

translate([0,0,0])
cylinder(d=2,h=4,$fn=16);

translate([0,0,0])
cylinder(d=2,h=4,$fn=16);

translate([L1/2,W1/2,0])
standoffh();
translate([L1/2,-W1/2,0])
standoffh();
translate([-L1/2,W1/2,0])
standoffh();
translate([-L1/2,-W1/2,0])
standoffh();
}

translate([-L1/2,W1/2,0])
standoff();
translate([L1/2,-W1/2,0])
standoff();
translate([-L1/2,-W1/2,0])
standoff();
translate([L1/2,W1/2,0])
standoff();

module standoff() {
    difference() {
        cylinder(d1=6,d2=4,h=5,$fn=64);
        cylinder(d=1.8,h=6,$fn=32);
    }
}

module standoff2() {
        cylinder(d1=6,d2=4,h=5,$fn=64);
        cylinder(d=2.1,h=6.5,$fn=32);
}

module standoffh() {

        cylinder(d=1.8,h=6,$fn=32);
}