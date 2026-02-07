package com.example.meepmeeptesting.Old;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedGoal_Old {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, 50.5, Math.toRadians(135)))
                //Back up to shoot
                .lineToX(-12)
                //Intake and shoot first line
                .turn(Math.toRadians(135))
                .lineToY(56, new TranslationalVelConstraint(30))
                .lineToY(14.5)
                .turn(Math.toRadians(-135))
                //Intake and shoot second line
                .turn(Math.toRadians(45))
                .lineToX(13.5)
                .turn(Math.toRadians(90))
                .lineToY(55, new TranslationalVelConstraint(30))
                .lineToY(14.5)
                .turn(Math.toRadians(-90))
                .lineToX(-12)
                .turn(Math.toRadians(-45))
                //Leave triangle and go close to gate
                .strafeTo(new Vector2d(0, 40))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
