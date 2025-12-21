package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 13.59)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, -49, Math.toRadians(225)))
                        .lineToX(-15)
                        .setTangent(-20)
                        .splineToLinearHeading(new Pose2d(-12, -43, Math.toRadians(270)), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-18.5, -18.5), Math.toRadians(225))
                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(12, -43, Math.toRadians(270)), Math.toRadians(270))
                        .setTangent(90)
                        .strafeToLinearHeading(new Vector2d(-18.5, -18.5), Math.toRadians(225))
                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(36, -28, Math.toRadians(270)), Math.toRadians(0))
                        .strafeTo(new Vector2d(36, -48))
                        .strafeToLinearHeading(new Vector2d(-18.5, -18.5), Math.toRadians(225))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}