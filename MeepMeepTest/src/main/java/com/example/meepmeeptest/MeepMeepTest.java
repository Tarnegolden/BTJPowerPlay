package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import org.jetbrains.annotations.NotNull;

import java.util.List;

import systems.GrabSystem;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        DefaultBotBuilder botBuilder = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.8, 18.4);

        GrabSystem grabSystem = new GrabSystem();

        RoadRunnerBotEntity myBot = botBuilder.followTrajectorySequence(drivee ->
                        drivee.trajectorySequenceBuilder(new Pose2d(35, 62.5, 0))
                                .strafeRight(62.5)
                                .back(5.5)
                                .addDisplacementMarker(grabSystem::drop)
                                .forward(5.5)

//                                    .addDisplacementMarker(grabSystem::flip)
//                                    .strafeLeft(12)
//                                    .forward(27.5)
//                                    .addDisplacementMarker(grabSystem::grab)
//                                    .back(27.5)
//                                    .strafeRight(12)
//                                    .addDisplacementMarker(grabSystem::flip)
//                                .back(5.5)
//                                .addDisplacementMarker(grabSystem::drop)
//                                .forward(5.5)

//                                    .addDisplacementMarker(grabSystem::flip)
//                                    .strafeLeft(12)
//                                    .forward(27.5)
//                                    .addDisplacementMarker(grabSystem::grab)
//                                    .back(27.5)
//                                    .strafeRight(12)
//                                    .addDisplacementMarker(grabSystem::flip)
//                                .back(5.5)
//                                .addDisplacementMarker(grabSystem::drop)
//                                .forward(5.5)
//
//                                    .addDisplacementMarker(grabSystem::flip)
//                                    .strafeLeft(12)
//                                    .forward(27.5)
//                                    .addDisplacementMarker(grabSystem::grab)
//                                    .back(27.5)
//                                    .strafeRight(12)
//                                    .addDisplacementMarker(grabSystem::flip)
//                                .back(5.5)
//                                .addDisplacementMarker(grabSystem::drop)
//                                .forward(5.5)
//
//                                    .addDisplacementMarker(grabSystem::flip)
//                                    .strafeLeft(12)
//                                    .forward(27.5)
//                                    .addDisplacementMarker(grabSystem::grab)
//                                    .back(27.5)
//                                    .strafeRight(12)
//                                    .addDisplacementMarker(grabSystem::flip)
//                                .back(5.5)
//                                .addDisplacementMarker(grabSystem::drop)
//                                .forward(5.5)
//
//                                    .addDisplacementMarker(grabSystem::flip)
//                                    .strafeLeft(12)
//                                    .forward(27.5)
//                                    .addDisplacementMarker(grabSystem::grab)
//                                    .back(27.5)
//                                    .strafeRight(12)
//                                    .addDisplacementMarker(grabSystem::flip)
//                                .back(5.5)
//                                .addDisplacementMarker(grabSystem::drop)
//                                .forward(5.5)

                                .strafeLeft(35)
                                .forward(-23)
                                .waitSeconds(2)
                                .build()
                );

//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .setDimensions(17.8, 18.4)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(35, 62.5, 0))
//                                .build()
//                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}