package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Systems {
    public GrabSystem grabSystem;

    public Systems(LinearOpMode opMode) {
        grabSystem = new GrabSystem(opMode);
    }
}
