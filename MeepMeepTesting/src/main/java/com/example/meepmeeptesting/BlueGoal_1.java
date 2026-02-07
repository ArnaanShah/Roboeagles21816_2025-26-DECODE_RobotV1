package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueGoal_1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, -50.5, Math.toRadians(-130)))
                        //Back up and shoot
                        .waitSeconds((2.5))
                        .lineToX(-24)
                        .waitSeconds((1.75*4))
                //Intake first line and shoot
                .strafeToLinearHeading(new Vector2d(-11.5, -20), Math.toRadians(-270))
                .strafeTo(new Vector2d(-11.5, -55), new TranslationalVelConstraint(30))
                        .strafeToLinearHeading(new Vector2d(-24, -24), Math.toRadians(-135))
                .waitSeconds((1.75*4))

                //Intake second line and shoot
                .strafeToLinearHeading(new Vector2d(13.5, -20), Math.toRadians(-270))
                        .strafeTo(new Vector2d(13.5, -50), new TranslationalVelConstraint(30))
                .strafeToLinearHeading(new Vector2d(-24, -24), Math.toRadians(-135))
                .waitSeconds((1.75*4))

                //Leave
                .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(-90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}