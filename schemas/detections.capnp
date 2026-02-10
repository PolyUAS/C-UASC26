@0xda0ae153d021b0bb;

struct Detection {
    ctrX @0 : UInt32;
    ctrY @1 : UInt32;
    width @2 : UInt32;
    height @3 : UInt32;

    enum Type {
        dropoff @0;
        pickup @1;
        waypoint @2;
    }

    type @4: Type;

    metadata @5: Text;
}


struct Frame {
    width @0 : UInt32;
    height @1 : UInt32;

    detections @2 : List(Detection);
    metadata @3: Text;
}
