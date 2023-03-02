package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class Trajectories {
    private final Systems systems;
    private final SampleMecanumDrive drive;
    private final int[] stackHeights = new int[]{410, 308, 205, 100, 0};

    public Trajectories(SampleMecanumDrive drive, Systems systems) {
        this.systems = systems;
        this.drive = drive;
    }

    public void BL() {
        Trajectory detection = drive.trajectoryBuilder(new Pose2d(35, 62.5, 0))
                .strafeRight(12.5).build();
        Trajectory toPole = drive.trajectoryBuilder(detection.end())
                .lineToConstantHeading(new Vector2d(35, 0)).build();

        drive.followTrajectory(detection);
        DetectionSystem.Signal signal = systems.detectionSystem.scan();
        drive.followTrajectory(toPole);

        for (int i = 0; i < 5; i++) {
            systems.elevatorSystem.goTo(ElevatorSystem.Level.HIGH);
            systems.pumpSystem.flip();
            dropBL();
            systems.pumpSystem.flip();
            systems.elevatorSystem.goTo(stackHeights[i]);

            Trajectory loopLeft = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(35, 12)).build();
            Trajectory loopForward = drive.trajectoryBuilder(loopLeft.end())
                    .forward(27).build();

            drive.followTrajectory(loopLeft);
            drive.followTrajectory(loopForward);
            systems.pumpSystem.collect();

            Trajectory loopBackward = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(27).build();
            Trajectory loopRight = drive.trajectoryBuilder(loopBackward.end())
                    .lineToConstantHeading(new Vector2d(35, 0)).build();

            drive.followTrajectory(loopBackward);
            drive.followTrajectory(loopRight);
        }

        systems.elevatorSystem.goTo(ElevatorSystem.Level.HIGH);
        systems.pumpSystem.flip();
        dropBL();
        systems.elevatorSystem.goTo(0);

        Trajectory toParking = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(35, 35)).build();
        drive.followTrajectory(toParking);

        switch (signal) {
            case ONE:
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(23).build());
                break;
            case THREE:
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(23).build());
        }
    }

    public void dropBL() {
        Trajectory backward = drive.trajectoryBuilder(new Pose2d()).back(5).build();
        Trajectory forward = drive.trajectoryBuilder(backward.end()).forward(5).build();
        drive.followTrajectory(backward);
        systems.pumpSystem.autoDrop();
        drive.followTrajectory(forward);
    }
}
