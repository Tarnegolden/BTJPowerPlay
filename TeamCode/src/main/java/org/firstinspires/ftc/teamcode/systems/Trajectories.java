package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class Trajectories {
    private final Trajectory dropBL;
    public final Trajectory goToDetectionBL;
    private final Pose2d blDrop = new Pose2d(35, 0, 0);
    private final Systems systems;
    private final SampleMecanumDrive drive;
    private final int[] stackHeights = new int[]{410, 308, 205, 100, 0};

    public Trajectories(SampleMecanumDrive drive, Systems systems) {
        this.systems = systems;
        this.drive = drive;

        goToDetectionBL = drive.trajectoryBuilder(new Pose2d(35, 62.5, 0))
//                .strafeRight(12.5)
//                .strafeLeft(12.5)
                .lineToConstantHeading(new Vector2d(35, 50))
                .build();

        dropBL = drive.trajectoryBuilder(blDrop)
                .splineToConstantHeading(new Vector2d(29.5, 0), 0)
                .addDisplacementMarker(systems.pumpSystem::autoDrop)
                .splineToConstantHeading(new Vector2d(35, 0), 0)
                .build();
    }

    public TrajectorySequence getFullTrajectoryBL(DetectionSystem.Signal signal) {
        PumpSystem pumpSystem = systems.pumpSystem;
        ElevatorSystem elevatorSystem = systems.elevatorSystem;

        TrajectorySequenceBuilder build = drive.trajectorySequenceBuilder(new Pose2d(35, 80, 0))
//        TrajectorySequenceBuilder build = drive.trajectorySequenceBuilder(goToDetectionBL.end())
                .addDisplacementMarker(() -> elevatorSystem.goTo(ElevatorSystem.Level.HIGH))
//                .addDisplacementMarker(pumpSystem::flip)
//                .strafeRight(50)
//                .strafeLeft(50)
                .lineToConstantHeading(new Vector2d(35, 0))
                .addTrajectory(dropBL);

        for (int i = 0; i < 5; i++) {
            build.addTrajectory(makeLoopBL(stackHeights[i])).addTrajectory(dropBL);
        }
        build.addDisplacementMarker(() -> elevatorSystem.goTo(ElevatorSystem.Level.GROUND));

        switch (signal) {
            case ONE:
                build.lineToConstantHeading(new Vector2d(35, 30))
                        .splineToConstantHeading(new Vector2d(58, 35), 0);
                break;
            case TWO:
                build.strafeLeft(35);
                break;
            case THREE:
                build.lineToConstantHeading(new Vector2d(35, 30))
                        .splineToConstantHeading(new Vector2d(12, 35), 0);
        }
        return build.build();
    }

    private Trajectory makeLoopBL(int i) {
        PumpSystem pumpSystem = systems.pumpSystem;
        ElevatorSystem elevatorSystem = systems.elevatorSystem;
        return drive.trajectoryBuilder(blDrop)
                .addDisplacementMarker(() -> elevatorSystem.goTo(i))
                .addDisplacementMarker(pumpSystem::flip)
                .lineTo(new Vector2d(35, 30))
                .splineTo(new Vector2d(60, 30), 0)
//                .strafeLeft(12)
//                .forward(27.5)
                .addDisplacementMarker(pumpSystem::collect)
                .addDisplacementMarker(() -> elevatorSystem.goTo(ElevatorSystem.Level.HIGH))
                .splineTo(new Vector2d(35, 30), 0)
                .splineTo(new Vector2d(35, 0), 0)
//                .back(27.5)
//                .strafeRight(12)
                .addDisplacementMarker(pumpSystem::flip)
                .build();
    }
}
