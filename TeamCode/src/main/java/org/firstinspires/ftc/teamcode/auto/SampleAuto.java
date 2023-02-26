package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.systems.DetectionSystem;
import org.firstinspires.ftc.teamcode.systems.GrabSystem;
import org.firstinspires.ftc.teamcode.systems.Systems;
import org.firstinspires.ftc.teamcode.systems.Trajectories;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name = "Sample Auto")
public class SampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Systems systems = new Systems(this);
        Trajectories trajectories = new Trajectories(drive, systems);

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajectories.getFullTrajectoryBL(DetectionSystem.Signals.ONE));
        telemetry.addData("Time: ", time);
        telemetry.update();
    }
}
