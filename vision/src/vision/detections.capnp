@0xda0ae153d021b0bb;

struct Detection {
    ctrX @0 : UInt32;
    ctrY @1 : UInt32;
    width @2 : UInt32;
    height @3 : UInt32;
    classId @4: Text;
    metadata @5: Text;
    confidence @6 : Float64;
}


struct Frame {
    width @0 : UInt32;
    height @1 : UInt32;

    detections @2 : List(Detection);
    metadata @3: Text;
}
