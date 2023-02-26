package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class GrabSystem {
    public GrabSystem() {}

    public GrabSystem(LinearOpMode opMode) {
        DcMotor motor = opMode.hardwareMap.get(DcMotor.class, "grab");
    }

    public void grab() {
        //
    }

    public void drop() {
        //
    }

    public void flip() {
        //
    }
}
