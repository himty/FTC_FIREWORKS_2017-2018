package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleopTest", group="FIREWORKS")
public class TeleopTest extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareTest robot = new HardwareTest();              // Use a K9'shardware
    ElapsedTime robotTime = new ElapsedTime();
    ElapsedTime ballDropperTime = new ElapsedTime(1000); //starting time is high so doesn't mess with timing
    ElapsedTime bPusherTime = new ElapsedTime(1000);

    final double MAX_JOYSTICK_VALUE = 1;

    double targetLeftPower;
    double targetRightPower;

    //Max is the bottom position for the jewel stick
    //Min is the upper position for the jewel stick
    //To make the stick move farther from the robot, raise this value
//    final double JEWEL_STICK_MAX = 0.56;
//    final double JEWEL_STICK_MIN = 0.07;

    final double DRIVE_POWER_CHANGE_MAX = 0.01;

    public boolean prevDPadDownState = false;

    //stores our instance of the Vuforia localization engine
    VuforiaLocalizerImplSubclass vuforia;

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //Make sure robot does not implement encoder
        robot.frontleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            doDriveTrain();
            //servoTest();
//            doLinearSlide();
//            RollOutArm();
            doJewelHitter();
            doLifter();
            doRampCollector();

            telemetry.addData("hi", "hello");
//            telemetry.addData("servoleft", robot.clawLeft.getPosition());
//            telemetry.addData("servoright", robot.clawRight.getPosition());
           telemetry.addData("Jewel Stick", robot.jewelStick.getPower());
            telemetry.addData("Lifter", robot.lifter.getPower());
//            telemetry.addData("Lifter Right", robot.lifterRight.getPosition());

            telemetry.addData("frontLeft", robot.frontleftMotor.getPower());
            telemetry.addData("frontRight", robot.frontrightMotor.getPower());
            telemetry.addData("backLeft", robot.backleftMotor.getPower());
            telemetry.addData("backRight", robot.backrightMotor.getPower());

            // Send telemetry message to signify robot waiting
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    /**
     * Makes the robot's drive train move
     */
    private void doDriveTrain() {
        //Defining which buttons on joystick corresponds to motor left and right power
        //Calculate left and right motor drivetrain power
//        double r = power * Path.Direction
//
//        private void moveForward(double power, double time) {
//            doDriveTrain(power, time, Math.atan2(power, 0) - Math.PI / 4);
//        }
//
//        moveForward(0.5);


        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        //Turning is configured on separate joystick
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

//        targetLeftPower = (gamepad1.right_stick_y - gamepad1.right_stick_x * 2);
//        targetRightPower = (gamepad1.right_stick_y + gamepad1.right_stick_x * 2);
//
//        //If bumper on joystick is not pressed, decrease motor power
//        if (!gamepad1.right_bumper) {
//            targetLeftPower /= 5;
//            targetRightPower /= 5;
//        }
//
//        if (Math.abs(targetLeftPower - robot.frontleftMotor.getPower()) > DRIVE_POWER_CHANGE_MAX) {
//            //and if the robot is trying to accelerate too much
//            if (targetLeftPower > robot.frontleftMotor.getPower()) {
//                targetLeftPower = robot.frontleftMotor.getPower() + DRIVE_POWER_CHANGE_MAX;
//            } else {
//                targetLeftPower = robot.frontleftMotor.getPower() - DRIVE_POWER_CHANGE_MAX;
//            }
//        }
//        if (Math.abs(targetRightPower - robot.frontrightMotor.getPower()) > DRIVE_POWER_CHANGE_MAX) {
//            if (targetRightPower > robot.frontrightMotor.getPower()) {
//                targetRightPower = robot.frontrightMotor.getPower() + DRIVE_POWER_CHANGE_MAX;
//            } else {
//                targetRightPower = robot.frontrightMotor.getPower() - DRIVE_POWER_CHANGE_MAX;
//            }
//        }
        //Set powers
        robot.frontleftMotor.setPower(v1 * 0.5);
        robot.frontrightMotor.setPower(v2 * 0.5);
        robot.backleftMotor.setPower(v3 * 0.5);
        robot.backrightMotor.setPower(v4 * 0.5);
    }

//    private void doClamping() {
//        if (gamepad1.dpad_up) {
//            //make claws clamp onto the object
//            //0.7 and 0.2 is range of values
//            robot.clawLeft.setPosition(0.7);
//            robot.clawRight.setPosition(0.2);
//        } else if (gamepad1.dpad_down) {
//            //make claws release the object
//            robot.clawLeft.setPosition(0.2);
//            robot.clawRight.setPosition(0.7);
//        } else {
//            //Make claws hang loose if there is nothing to do
//            robot.clawLeft.setPosition(0.5);
//            robot.clawRight.setPosition(0.5);
//        }
//    }
//
//    private void servoTest() {
//        float servoPos = gamepad2.left_stick_y;
//
//        servoPos = (servoPos * 0.5f) + 0.5f;
//
//        servoPos = Range.clip(servoPos, 0, 1);
//
//        robot.clawLeft.setPosition(servoPos);
//        robot.clawRight.setPosition(servoPos);
//    }

    /**
     * Linear slide moves up and down based on pressing the right joystick
     */
//    private void doLinearSlide() {
//        robot.linearSlide.setPower(gamepad2.right_stick_y * -1);
//    }

    /**
     * Horizontal lift comes out by pressing left joystick
     */
//    private void RollOutArm() {
//        robot.rollArm.setPower(gamepad2.left_stick_y);
//    }

    /**
     * The Jewel Stick hits one of the balls based on the team color
     * This method lowers and raises the stick
     * Jewel Stick travels in Increments of 0.15 (speed)
     */
    private void doJewelHitter() {
        if (gamepad2.dpad_down) {
            robot.jewelStick.setPower(-1);
        }
        if (gamepad2.dpad_up) {
            robot.jewelStick.setPower(1);
        }
    }

    private void doRampCollector() {
        robot.rampLeft.setPower(gamepad2.left_stick_y);
        robot.rampRight.setPower(gamepad2.left_stick_y);
    }
//
//    private void doJewelHitter() {
//        //double currPosition = robot.jewelStick.getPosition();
//        if (gamepad2.dpad_down) {
//            //make jewelstick go down
//            robot.jewelStick.setPosition(JEWEL_STICK_MAX);
//
//        } else if (gamepad2.dpad_up) {
//            //make jewelstick go up
//            robot.jewelStick.setPosition(JEWEL_STICK_MIN);
//        } else {
//            //Make jewel stick hang loose if there is nothing to do
//            robot.jewelStick.setPosition(0.01);
//        }

    private void doLifter() {
        robot.lifter.setPower(gamepad2.right_stick_y * 0.3);
//        if (gamepad2.right_stick_y) {
//            robot.lifter.setPower(0.5);
//        } else if (gamepad1.dpad_down) {
//            robot.lifter.setPower(-0.5);
//        } else {
//            robot.lifter.setPower(0);
//        }
//    }
    }
}
//        robot.lifter.setPower(gamepad1.dpad_up);
//        if (gamepad1.dpad_up) {
//            robot.lifterLeft.setPosition(0.3);
//            robot.lifterRight.setPosition(0.3);
//        } else if (gamepad1.dpad_down) {
//            robot.lifterLeft.setPosition(0.8);
//            robot.lifterRight.setPosition(0.8);
//        } else {
//            robot.lifterLeft.setPosition(0.5);
//            robot.lifterRight.setPosition(0.5);
//        }

//    }



