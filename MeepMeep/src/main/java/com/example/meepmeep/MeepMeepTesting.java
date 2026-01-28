package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        Vector2d shootVector = new Vector2d(-35, 28);
        double shootHeading = Math.toRadians(125);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 13.59)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, 61, Math.toRadians(90)))
                .lineToY(50)
                .splineToSplineHeading(new Pose2d(shootVector, shootHeading), Math.toRadians(270-125))
                .splineToLinearHeading(new Pose2d(new Vector2d(16.5, 60), Math.toRadians(127)), Math.toRadians(127))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}