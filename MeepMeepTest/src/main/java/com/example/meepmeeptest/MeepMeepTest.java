package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

        RoadRunnerBotEntity myBot = botBuilder.followTrajectorySequence(drivee ->
                        drivee.trajectorySequenceBuilder(new Pose2d(35, 62.5, 0))
                                .strafeRight(12.5)
                                .waitSeconds(1)
                                .strafeRight(50)
//                                .strafeRight(62.5)
//                                .back(5.5)
//                                .forward(5.5)

//                                    .strafeLeft(12)
//                                    .forward(27.5)
//                                    .back(27.5)
//                                    .strafeRight(12)
//                                .back(5.5)
//                                .forward(5.5)
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