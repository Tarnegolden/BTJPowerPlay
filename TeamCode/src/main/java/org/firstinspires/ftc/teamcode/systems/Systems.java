package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Systems {
    public PumpSystem pumpSystem;
    public DetectionSystem detectionSystem;
    public ElevatorSystem elevatorSystem;

    public Systems(LinearOpMode opMode) {
        pumpSystem = new PumpSystem(opMode);
        detectionSystem = new DetectionSystem(opMode);
        elevatorSystem = new ElevatorSystem(opMode);
    }
}
