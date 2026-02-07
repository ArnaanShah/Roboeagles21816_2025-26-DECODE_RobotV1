package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.PoseStorage;

@Config
@Autonomous(name = "Blue Goal 1- Shoot loaded, line 1, leave near gate", group = "Autonomous")

public class BlueGoal_1 extends LinearOpMode {
    public class Outtake {
        private DcMotorEx outtake;
        private CRServo ballPusher;
        private Servo ballStopper;
        private double MaxVelocity = (6000 / 60.0) * 28; //RPM divided by 60 seconds times 28 ticks per revolution


        public Outtake(HardwareMap hardwareMap) {
            outtake = hardwareMap.get(DcMotorEx.class, "outtake");
            outtake.setDirection(DcMotorEx.Direction.REVERSE);
            outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


            ballPusher = hardwareMap.get(CRServo.class, "ballPusher");
            ballStopper = hardwareMap.get(Servo.class, "ballStopper");

            ballStopper.setPosition(0.5);
        }
/*
        public class ShootArtifacts implements Action {
            private int Amount;
            private double MaxVelocity = (6000 / 60.0) * 28; //RPM divided by 60 seconds times 28 ticks per revolution
            private double Velocity;
            private boolean initialized = false;

            public ShootArtifacts(double Power, int Amount) {
                this.Velocity = Power*MaxVelocity;
                this.Amount = Amount;
            }


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                outtake.setVelocity(Velocity);
                for (int i = 1; i <= Amount; i++) {
                    ballStopper.setPosition(0.8);
                    sleep(500);
                    ballPusher.setPower(0.75);
                    sleep(500);
                    ballStopper.setPosition(0.5);
                    ballPusher.setPower(0);
                    sleep(750);
                }

                return false;
            }
        }

        public Action shootArtifacts(double power, int amount) {
            return new ShootArtifacts(power, amount);
        }*/

        public class OpenStopper implements Action {
           private double Position = 0.8;
           private boolean initialized = false;



            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                ballStopper.setPosition(Position);

                return false;
            }
        }

        public Action openStopper() {
            return new OpenStopper();
        }


        public class CloseStopper implements Action {
            private double Position = 0.5;
            private boolean initialized = false;



            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                ballStopper.setPosition(Position);

                return false;
            }
        }

        public Action closeStopper() {
            return new CloseStopper();
        }

        public class MovePusher implements Action {
            private double Power = 0.75;
            private boolean initialized = false;



            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                ballPusher.setPower(Power);

                return false;
            }
        }

        public Action movePusher() {
            return new MovePusher();
        }

        public class StopPusher implements Action {
            private double Power = 0;
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                ballPusher.setPower(Power);

                return false;
            }
        }

        public Action stopPusher() {
            return new StopPusher();
        }


        public class RampUpOuttake implements Action {
            private double GoalVelocity;
            private boolean initialized = false;

            public RampUpOuttake(double Power) {
                this.GoalVelocity = Power*MaxVelocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtake.setVelocity(GoalVelocity);
                    initialized = true;
                }

                double vel = outtake.getVelocity();
                packet.put("outtakeVelocity", vel);

                return (vel < GoalVelocity*0.5);

            }
        }

        public Action rampUpOuttake(double power) {
            return new RampUpOuttake(power);
        }


        /*
        public class ShootArtifacts implements Action {

            private int step = 0;
            private long stepStartTime = 0;

            private boolean elapsed(long ms) {
                return System.currentTimeMillis() - stepStartTime >= ms;
            }


            @Override
            public boolean run(TelemetryPacket packet) {

                switch (step) {

                    case 0:
                        rampUpOuttake(0.6);
                        stepStartTime = System.currentTimeMillis();
                        step++;
                        break;

                    case 1:
                        if (elapsed(500)) {          // wait 0.5s
                            openStopper();
                            stepStartTime = System.currentTimeMillis();
                            step++;
                        }
                        break;

                    case 2:
                        if (elapsed(300)) {          // wait 0.3s
                            movePusher();
                            stepStartTime = System.currentTimeMillis();
                            step++;
                        }
                        break;

                    case 3:
                        if (elapsed(400)) {          // wait 0.4s
                            stopPusher();
                            closeStopper();
                            stepStartTime = System.currentTimeMillis();
                            step++;
                        }
                        break;

                    case 4:
                        step = 0;
                        return true;
                }

                return false;
            }

        }*/


        public Action shootArtifacts(double power, int amount) {
            Action[] shots = new Action[amount];
            for (int i = 0; i < amount; i++) {
                if (i>=3){
                    power -= 0.0015;
                }
                shots[i] = new SequentialAction(
                        rampUpOuttake(power),
                        new SleepAction(0.2),
                        openStopper(),
                        new SleepAction(0.5),
                        movePusher(),
                        new SleepAction(0.5),
                        stopPusher(),
                        closeStopper(),
                        new SleepAction(0.75)
                );
            }

            return new SequentialAction(shots);
        }

    }


    public class Intake {
        private DcMotor intake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotor.Direction.REVERSE);

        }

        public class IntakeOn implements Action {
            private double intakeSpeed = 1;
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                intake.setPower(intakeSpeed);

                return false;
            }
        }

        public Action intakeOn() {
            return new IntakeOn();
        }

    }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-50, -50.5, Math.toRadians(-130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Outtake outtake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // actionBuilder builds from the drive steps passed to it

        TrajectoryActionBuilder leg1 = drive.actionBuilder(initialPose)
                //Back up and shoot
                .waitSeconds(1)
                .lineToX(-24);


        TrajectoryActionBuilder leg2 = leg1.endTrajectory().fresh()
                //Intake first line and shoot
                .strafeToLinearHeading(new Vector2d(-4.5, -20), Math.toRadians(-270))
                .strafeTo(new Vector2d(-5.5, -60))
                .strafeToLinearHeading(new Vector2d(-20, -20), Math.toRadians(-140));


        TrajectoryActionBuilder leg3 = leg2.endTrajectory().fresh()

                //Intake second line and shoot
                .strafeToLinearHeading(new Vector2d(10, -20), Math.toRadians(-270))
                .strafeTo(new Vector2d(10, -55), new TranslationalVelConstraint(30))
                .strafeToLinearHeading(new Vector2d(-24, -24), Math.toRadians(-130));

        Action leg4 = leg2.endTrajectory().fresh()
                //Leave
                .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(-90))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeOn(),
                        outtake.rampUpOuttake(0.43),

                        leg1.build(),
                        outtake.shootArtifacts(0.46, 4),

                        leg2.build(),
                        outtake.shootArtifacts(0.42, 4),
                        //leg3.build(),
                        //outtake.shootArtifacts(0.17, 4),
                        leg4
                )

        );

        PoseStorage.pose = drive.localizer.getPose();

    }
}
