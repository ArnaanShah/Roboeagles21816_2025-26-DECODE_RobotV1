package org.firstinspires.ftc.teamcode.Old;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous(name="Red Goal 1 - Old")
@Disabled
public class RedGoal_1_Old extends LinearOpMode {

    private DcMotor outtake   = null;
    private CRServo ballPusher   = null;
    private Servo ballStopper   = null;
    private DcMotor intake   = null;





    public class shootArtifacts implements InstantFunction {
        private int amount = 4;
        private double power = 0.7;

        public shootArtifacts(double power, int amount){
            this.power = power;
            this.amount = amount;
        }

        public void run(){
            outtake.setPower(power);
            for (int i = 1; i <= amount; i++) {
                ballStopper.setPosition(0.8);
                sleep(500);
                ballPusher.setPower(0.75);
                sleep(500);
                ballStopper.setPosition(0.5);
                ballPusher.setPower(0);
                sleep(750);
            }

        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        ballPusher = hardwareMap.get(CRServo.class, "ballPusher");
        ballStopper = hardwareMap.get(Servo.class, "ballStopper");

        intake = hardwareMap.get(DcMotor.class, "intake");

        outtake.setDirection(DcMotor.Direction.REVERSE);

        ballStopper.setPosition(0.5);

        intake.setDirection(DcMotor.Direction.REVERSE);

        intake.setPower(1);
        outtake.setPower(0.7);
        sleep(2500);



        Pose2d beginPose = new Pose2d(new Vector2d( -50, 50.5), Math.toRadians(130));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                //Back up and shoot
                .lineToX(-24)
                .stopAndAdd(new shootArtifacts(0.7, 4))

                //Intake first line and shoot
                .strafeToLinearHeading(new Vector2d(-11.5, 20), Math.toRadians(270))
                .strafeTo(new Vector2d(-11.5, 50), new TranslationalVelConstraint(30))
                .strafeToLinearHeading(new Vector2d(-24, 24), Math.toRadians(135))
                .stopAndAdd(new shootArtifacts(0.7, 4))

                //Intake second line and shoot
                .strafeToLinearHeading(new Vector2d(13.5, 20), Math.toRadians(270))
                .strafeTo(new Vector2d(13.5, 50), new TranslationalVelConstraint(30))
                .strafeToLinearHeading(new Vector2d(-24, 24), Math.toRadians(135))
                .stopAndAdd(new shootArtifacts(0.7, 4))

                //Leave
                .strafeToLinearHeading(new Vector2d(0, 40), Math.toRadians(90))

                .build();

        Actions.runBlocking(new SequentialAction(path));


    }
}