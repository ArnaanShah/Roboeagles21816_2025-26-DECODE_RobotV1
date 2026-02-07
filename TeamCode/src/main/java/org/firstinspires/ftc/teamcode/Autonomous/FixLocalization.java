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
@Autonomous(name = "Fix Localization by starting at red side", group = "Autonomous")

public class FixLocalization extends LinearOpMode {
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

        public Action shootArtifacts(double power, int amount) {
            Action[] shots = new Action[amount];
            for (int i = 0; i < amount; i++) {
                if (i>=3){
                    power -= 0.0015;
                }
                shots[i] = new SequentialAction(
                        rampUpOuttake(power),
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
        Pose2d initialPose = new Pose2d(61.5, 14.5, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Outtake outtake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // actionBuilder builds from the drive steps passed to it

        TrajectoryActionBuilder leg1 = drive.actionBuilder(initialPose)
                .waitSeconds((22.5))//Ramp up ot Wait 15s
                //Forward and turn to shoot
                .strafeToLinearHeading(new Vector2d(52, 14.5), Math.toRadians(155));
        //Shoot 4


        TrajectoryActionBuilder leg2 = leg1.endTrajectory().fresh()
                //Intake and shoot corner three
                .strafeToLinearHeading(new Vector2d(40, 77.5), Math.toRadians(180))
                .strafeTo(new Vector2d(64, 77.5), new TranslationalVelConstraint(35))
                .strafeToLinearHeading(new Vector2d(52, 10), Math.toRadians(155));
                //Shoot 4

        TrajectoryActionBuilder leg3 = leg2.endTrajectory().fresh()
                //Intake and shoot third line
                .strafeToLinearHeading(new Vector2d(36, 30.5), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(36, 61), Math.toRadians(270), new TranslationalVelConstraint(30))
                .strafeToLinearHeading(new Vector2d(52, 14.5), Math.toRadians(155));
                //Shoot 4

        Action leg4 = leg1.endTrajectory().fresh()
                //Leave
                .strafeToLinearHeading(new Vector2d(50, 40), Math.toRadians(90))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        //intake.intakeOn(),
                        //outtake.rampUpOuttake(0.235),

                        //leg1.build(),
                        //outtake.shootArtifacts(0.24, 4),
                        //leg2.build(),
                        //outtake.shootArtifacts(0.24, 4),
                        //leg3.build(),
                        //outtake.shootArtifacts(0.225, 4),
                        //leg4
                )
        );

        PoseStorage.pose = drive.localizer.getPose();


    }
}
