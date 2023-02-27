package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.systems.DetectionSystem;
import org.firstinspires.ftc.teamcode.systems.Systems;
import org.firstinspires.ftc.teamcode.systems.Trajectories;

@TeleOp(name = "Sample Auto")
public class SampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Systems systems = new Systems(this);
        Trajectories trajectories = new Trajectories(drive, systems);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectories.goToDetectionBL);
        DetectionSystem.Signal s = systems.detectionSystem.scan();
        telemetry.addData("Signal: ", s);
        telemetry.update();

        drive.followTrajectorySequence(trajectories.getFullTrajectoryBL(s));
        telemetry.addData("Time: ", time);
        telemetry.update();
    }
}
