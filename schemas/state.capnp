@0x91bffa14f379b39b;

struct HilStateQuaternion {
    timeUsec @0 : UInt64;

    struct Quaternion {
        w @0 : Float32;
        x @1 : Float32;
        y @2 : Float32;
        z @3 : Float32;
    }


    attitudeQuaternion @1 : Quaternion;
    rollspeed @2: Float32;
    pitchspeed @3 : Float32;
    yawspeed @4 : Float32;
    lat @5 : Int32;
    lon @6 : Int32;
    alt @7 : Int32;
    vx @8 : Int16;
    vy @9 : Int16;
    vz @10 : Int16;
    indAirspeed @11 : UInt16;
    trueAirspeed @12 : UInt16;

    xacc @13 : Int16;
    yacc @14 : Int16;
    zacc @15 : Int16;
}
