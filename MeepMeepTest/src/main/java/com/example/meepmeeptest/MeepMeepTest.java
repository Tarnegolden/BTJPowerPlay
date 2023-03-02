package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        DefaultBotBuilder botBuilder = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.8, 18.4);

        RoadRunnerBotEntity myBot = botBuilder.followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 62.5, 0))
                                .strafeRight(12.5)
                                .lineToConstantHeading(new Vector2d(35, 0))
                                .waitSeconds(1)
                                .back(5)
                                .waitSeconds(0.5)
                                .forward(5)
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(35, 12))
                                .forward(27)
                                .waitSeconds(0.5)
                                .back(27)
                                .lineToConstantHeading(new Vector2d(35, 0))
                                .waitSeconds(1)
                                .back(5)
                                .waitSeconds(0.5)
                                .forward(5)
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(35, 35))
                                .forward(23)

                                .waitSeconds(2)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}