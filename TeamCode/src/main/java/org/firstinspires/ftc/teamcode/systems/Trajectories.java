package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class Trajectories {
    private final Trajectory loopBL;
    private final Trajectory drop;
//    private final Systems systems;
    private final SampleMecanumDrive drive;

    public Trajectories(SampleMecanumDrive drive, Systems systems) {
//        this.systems = systems;
        this.drive = drive;
        GrabSystem grabSystem = systems.grabSystem;

        drop = drive.trajectoryBuilder(new Pose2d(35, 0, 0))
                .back(5.5)
                .addDisplacementMarker(grabSystem::drop)
                .forward(5.5)
                .build();

        loopBL = drive.trajectoryBuilder(new Pose2d(35, 0, 0))
                .addDisplacementMarker(grabSystem::flip)
                .strafeLeft(12)
                .forward(27.5)
                .addDisplacementMarker(grabSystem::grab)
                .back(27.5)
                .strafeRight(12)
                .addDisplacementMarker(grabSystem::flip)
                .build();
    }

    public TrajectorySequence getFullTrajectoryBL(DetectionSystem.Signals signal) {
        TrajectorySequenceBuilder build = drive.trajectorySequenceBuilder(new Pose2d(35, 62.5, 0))
                .strafeRight(62.5)
                .addTrajectory(drop);

        for (int i = 0; i < 5; i++) {
            build.addTrajectory(loopBL).addTrajectory(drop);
        }

        switch (signal) {
            case ONE:
                build.forward(23);
                break;
            case TWO:
                build.strafeLeft(35);
                break;
            case THREE:
                build.back(23);
        }

        return build.build();
    }
}
