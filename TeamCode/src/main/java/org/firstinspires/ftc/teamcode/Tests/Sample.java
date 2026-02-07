package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous
@Disabled
public class Sample extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initPos = new Pose2d(-52,-52,Math.toRadians(225));
        Pose2d goalPos = new Pose2d(-18,-18, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPos);

        TrajectoryActionBuilder samplePath = drive.actionBuilder(initPos)
                        .lineToX(-18);

        TrajectoryActionBuilder samplePath2 = drive.actionBuilder(goalPos)
                        .splineToLinearHeading(new Pose2d(12, -36, Math.toRadians(270)), Math.toRadians(270));

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(samplePath.build(), new SleepAction(2),
                        samplePath2.build())
        );

    }
}
