/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name="Teleop - WEBCAM, INTAKE, OUTTAKE", group="Linear OpMode")
//@Disabled
public class TeleopAllFeatures extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;


    private DcMotor outtakeMotor = null;
    private CRServo ballPusher = null;
    private Servo ballStopper = null;
    private DcMotor intakeMotor = null;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "LF");
        backLeftDrive = hardwareMap.get(DcMotor.class, "LB");
        frontRightDrive = hardwareMap.get(DcMotor.class, "RF");
        backRightDrive = hardwareMap.get(DcMotor.class, "RB");

        outtakeMotor = hardwareMap.get(DcMotor.class, "outtake");
        ballPusher = hardwareMap.get(CRServo.class, "ballPusher");
        ballStopper = hardwareMap.get(Servo.class, "ballStopper");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");



        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        ballPusher.setDirection(CRServo.Direction.FORWARD);

        ballStopper.setPosition(0.5);
        //ballPusher.setPower(1);

        initAprilTag();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double drivetrainSpeed = 0.8;

        boolean lastUpPressed2 = false; //Increase speed
        boolean lastDownPressed2 = false;//Decrease speed
        boolean lastAPressed2 = false;//Motor direction to Forward
        boolean lastBPressed2 = false;//Motor direction to Reverse
        boolean lastXPressed2 = false;//Set to 0.75
        boolean lastYPressed2 = false;//Set to 0
        boolean lastLeftBumper2 = false; //Move the gate
        boolean lastRightBumper2 = false; //Adjust and Shoot

        boolean lastAPressed1 = false; //slow
        boolean lastBPressed1 = false; //medium
        boolean lastYPressed1 = false; //high
        boolean lastUpPressed1 = false; //Forward intake
        boolean lastDownPressed1 = false; //Reverse intake
        boolean lastBackPressed1 = false; //Turn off intake
        boolean lastStartPressed1 = false; //Turn on intake

        double outtakePower = 0;
        double intakePower = 1;
        boolean intakeForward = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //ballPusher.setPower(1);

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            //Adjust these variables to change the speed of the motors

            boolean upPressed2 = gamepad2.dpad_up;
            boolean downPressed2 = gamepad2.dpad_down;
            boolean leftBumper2 = gamepad2.left_bumper;
            boolean rightBumper2 = gamepad2.right_bumper;
            boolean aPressed2 = gamepad2.a;
            boolean bPressed2 = gamepad2.b;
            boolean xPressed2 = gamepad2.x;
            boolean yPressed2 = gamepad2.y;

            boolean aPressed1 = gamepad1.a;
            boolean bPressed1 = gamepad1.b;
            boolean yPressed1 = gamepad1.y;
            boolean upPressed1 = gamepad1.dpad_up;
            boolean downPressed1 = gamepad1.dpad_down;
            boolean backPressed1 = gamepad1.back;
            boolean startPressed1 = gamepad1.start;

            //Change the outtake
            if (upPressed2 && !lastUpPressed2){ //UP
                outtakePower += 0.05;
            } else if (downPressed2 && !lastDownPressed2) { //DOWN
                outtakePower -= 0.05;
            } else if (xPressed2 && !lastXPressed2) { //STRAIGHT TO 0.75
                outtakePower = 0.75;
            } else if (yPressed2 && !lastYPressed2) { //OFF
                outtakePower = 0;
            } else if (aPressed2 && !lastAPressed2) {//REVERSE DIRECTION BACKWARD
                outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            } else if (bPressed2 && !lastBPressed2) {//REVERSE DIRECTION FORWARD
                outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            lastUpPressed2 = upPressed2;
            lastDownPressed2 = downPressed2;

            lastAPressed2 = aPressed2;
            lastBPressed2 = bPressed2;


            //To move the gate
            if (leftBumper2&&!lastLeftBumper2) {
                ballStopper.setPosition(1.0);
                sleep(300);
                ballPusher.setPower(0.8);
                sleep(650);
                ballStopper.setPosition(0.5);
                ballPusher.setPower(0);
                sleep(250);
            }

            lastLeftBumper2 = leftBumper2;

            //SPIT BALLS OUT OF INTAKE
            if((downPressed1&&!lastDownPressed1) && intakeForward) {
                intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                ballPusher.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeForward = false;
            }
            //PUSH BALLS IN TO INTAKE
            else if ((upPressed1 && !lastUpPressed1) && !intakeForward) {
                intakeMotor.setDirection(DcMotor.Direction.REVERSE);
                ballPusher.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeForward = true;
            }

            lastDownPressed1 = downPressed1;
            lastUpPressed1 = upPressed1;

            if (startPressed1 && !lastStartPressed1){
                intakePower = 1;
            } else if (backPressed1 && !lastBackPressed1) {
                intakePower = 0;
            }

            lastStartPressed1 = startPressed1;
            lastBackPressed1 = backPressed1;

            //To slow down the movement of the drivetrain
            if(aPressed1 && !lastAPressed1){
                drivetrainSpeed = 0.3;
            } else if (bPressed1 && !lastBPressed1) {
                drivetrainSpeed = 0.8;
            } else if (yPressed1 && !lastYPressed1) {
                drivetrainSpeed = 0.95;
            }

            lastAPressed1 = aPressed1;
            lastBPressed1 = bPressed1;

            // Get AprilTag detections ONCE per loop
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            // Telemetry using the same list
            telemetryAprilTag(currentDetections);


            AprilTagDetection desiredTag = null; //Store useful tag

            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == 24) {
                    desiredTag = detection; // Found the target tag, store it
                    break; // Exit the loop as we found what we needed
                } else if (detection.id == 20) {
                    desiredTag = detection;
                    break;
                }
            }

            //Adjust robot to face april tag
            if ((rightBumper2 && !lastRightBumper2) && desiredTag != null){
                outtakePower = prepareToShoot(currentDetections);
            }

            lastRightBumper2 = rightBumper2;

            //Allow driver to track april tag
            if (gamepad1.x && desiredTag != null){
                if (desiredTag.ftcPose.bearing > 3){
                    frontLeftPower = -0.3;
                    backLeftPower = -0.3;
                    frontRightPower = 0.3;
                    backRightPower = 0.3;
                } else if (desiredTag.ftcPose.bearing < -3) {
                    frontLeftPower = 0.3;
                    backLeftPower = 0.3;
                    frontRightPower = -0.3;
                    backRightPower = -0.3;
                }
            }

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            if (outtakePower > 1){
                outtakePower = 1;
            } else if (outtakePower < 0) {
                outtakePower = 0;
            }


            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower*drivetrainSpeed);
            frontRightDrive.setPower(frontRightPower*drivetrainSpeed);
            backLeftDrive.setPower(backLeftPower*drivetrainSpeed);
            backRightDrive.setPower(backRightPower*drivetrainSpeed);


            outtakeMotor.setPower(outtakePower);

            intakeMotor.setPower(intakePower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right:", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right:", "%4.2f, %4.2f", backLeftPower, backRightPower);

            telemetry.addData("Outtake Power:", "%4.2f", outtakePower);
            telemetry.addData("Outtake Direction:", "position: " + outtakeMotor.getDirection());
            telemetry.addData("Intake Direction:", "position: " + intakeMotor.getDirection());

            telemetry.update();
        }


        visionPortal.close();
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag(List<AprilTagDetection> currentDetections) {

        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("Y %6.1f  (inch)", detection.ftcPose.y));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)",
                        detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey: Y = Forward dist.");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    private double prepareToShoot(List<AprilTagDetection> ignoredInput) {

        sleep(500);

        while (opModeIsActive() && !gamepad2.right_bumper) {

            List<AprilTagDetection> newDetections = aprilTag.getDetections();
            AprilTagDetection goalTag = null;

            for (AprilTagDetection detection : newDetections) {
                if (detection.id == 24) {
                    goalTag = detection; // Found the target tag, store it
                    break; // Exit the loop as we found what we needed
                } else if (detection.id == 20) {
                    goalTag = detection;
                    break;
                }
            }

            if (goalTag == null) {
                frontLeftDrive.setPower(0);
                backLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backRightDrive.setPower(0);

                telemetry.addLine("CANNOT SEE TAG");
                telemetry.update();
                continue;   // try again next time
            }

            // align to goal
            if (goalTag.ftcPose.bearing > 3) {
                frontLeftDrive.setPower(-0.25);
                backLeftDrive.setPower(-0.25);
                frontRightDrive.setPower(0.25);
                backRightDrive.setPower(0.25);
            }
            else if (goalTag.ftcPose.bearing < -3) {
                frontLeftDrive.setPower(0.25);
                backLeftDrive.setPower(0.25);
                frontRightDrive.setPower(-0.25);
                backRightDrive.setPower(-0.25);
            }
            else {
                break;
            }
        }

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        List<AprilTagDetection> finalDetections = aprilTag.getDetections();
        AprilTagDetection tag = null;

        for (AprilTagDetection d : finalDetections) {
            if (d.id == 24 || d.id == 20) {
                tag = d;
                break;}
        }

        if (tag == null) return 0.75;

        //calculate power from distance
        double firstValue = (1.0/260.0) * tag.ftcPose.y + (0.57);
        return Math.round(firstValue * 20.0) / 20.0;
    }


}
