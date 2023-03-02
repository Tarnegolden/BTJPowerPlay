package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Systems {
    public PumpSystem pumpSystem;
    public DetectionSystem detectionSystem;
    public ElevatorSystem elevatorSystem;
    public SampleMecanumDrive drive;

    public Systems(LinearOpMode opMode, SampleMecanumDrive drive) {
        detectionSystem = new DetectionSystem(opMode);
        elevatorSystem = new ElevatorSystem(opMode);
        pumpSystem = new PumpSystem(opMode, this);
        this.drive = drive;
    }

    public Systems(LinearOpMode opMode) {
        detectionSystem = new DetectionSystem(opMode);
        elevatorSystem = new ElevatorSystem(opMode);
        pumpSystem = new PumpSystem(opMode, this);
        this.drive = new SampleMecanumDrive(opMode.hardwareMap);
    }
}
