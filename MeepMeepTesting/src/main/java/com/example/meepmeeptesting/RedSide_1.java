package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedSide_1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61.5, 14.5, Math.toRadians(180)))
                //Forward to shoot
                .waitSeconds((2.5))//Ramp up ot
                .strafeToLinearHeading(new Vector2d(52, 14.5), Math.toRadians(155))
                .waitSeconds((1.75*4))//Shoot 4
                //Intake and shoot corner three
                .strafeToLinearHeading(new Vector2d(40, 77.5), Math.toRadians(180))
                .strafeTo(new Vector2d(64, 77.5))
                //.strafeTo(new Vector2d(61, 60.5))
                //.waitSeconds(0.25)
                //.strafeTo(new Vector2d(61, 67.5))
                .strafeToLinearHeading(new Vector2d(52, 12), Math.toRadians(155))
                .waitSeconds((1.75*4))//Shoot 4


                //Intake and shoot third line
                .strafeToLinearHeading(new Vector2d(36, 30.5), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(36, 61), Math.toRadians(270), new TranslationalVelConstraint(30))
                .strafeToLinearHeading(new Vector2d(52, 14.5), Math.toRadians(155))
                .waitSeconds((1.75*4))//Shoot 4
                //Leave triangle
                        .strafeToLinearHeading(new Vector2d(50, 40), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}