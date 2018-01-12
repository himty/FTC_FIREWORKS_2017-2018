package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleopTest", group="FIREWORKS")
public class TeleopTest extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareTest    robot               = new HardwareTest();              // Use a K9'shardware
    ElapsedTime     robotTime           = new ElapsedTime();
    ElapsedTime     ballDropperTime     = new ElapsedTime(1000); //starting time is high so doesn't mess with timing
    ElapsedTime     bPusherTime         = new ElapsedTime(1000);

    final double MAX_JOYSTICK_VALUE = 1;

    double targetLeftPower;
    double targetRightPower;

    //Max is the bottom position for the jewel stick
    //Min is the upper position for the jewel stick
    //To make the stick move farther from the robot, raise this value
    final double JEWEL_STICK_MAX = 0.56;
    final double JEWEL_STICK_MIN = 0.07;

    //stores our instance of the Vuforia localization engine
    VuforiaLocalizerImplSubclass vuforia;

    @Override
    public void runOpMode(){
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
            doClamping();
            doLinearSlide();
            RollOutArm();
            doJewelHitter();

            telemetry.addData("hi", "hello");
            telemetry.addData("servoleft", robot.clawLeft.getPosition());
            telemetry.addData("servoright", robot.clawRight.getPosition());
            telemetry.addData("Jewel Stick", robot.jewelStick.getPosition());

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
        targetLeftPower = (gamepad1.right_stick_y - gamepad1.right_stick_x *2);
        targetRightPower = (gamepad1.right_stick_y + gamepad1.right_stick_x * 2);

        //If bumper on joystick is not pressed, decrease motor power
        if (!gamepad1.right_bumper){
            targetLeftPower /= 5;
            targetRightPower /= 5;
        }

        //Set powers
        robot.frontleftMotor.setPower(targetLeftPower);
        robot.backleftMotor.setPower(targetLeftPower);
        robot.frontrightMotor.setPower(targetRightPower);
        robot.backrightMotor.setPower(targetRightPower);
    }


    private void doClamping() {
        if (gamepad1.dpad_up) {
            //make claws clamp onto the object
            //0.7 and 0.2 is range of values
            robot.clawLeft.setPosition(0.7);
            robot.clawRight.setPosition(0.2);
        } else if (gamepad1.dpad_down) {
            //make claws release the object
            robot.clawLeft.setPosition(0.2);
            robot.clawRight.setPosition(0.7);
        }
        else {
            //Make claws hang loose if there is nothing to do
            robot.clawLeft.setPosition(0.5);
            robot.clawRight.setPosition(0.5);
        }
    }

    /**
     * Linear slide moves up and down based on pressing the right joystick
     */
    private void doLinearSlide() {
        robot.linearSlide.setPower(gamepad2.right_stick_y * -1);
    }

    /**
     * Horizontal lift comes out by pressing left joystick
     */
    private void RollOutArm() {
        robot.RollArm.setPower(gamepad2.left_stick_y);
    }

    /**
     * The Jewel Stick hits one of the balls based on the team color
     * This method lowers and raises the stick
     * Jewel Stick travels in Increments of 0.15 (speed)
     */
    private void doJewelHitter() {
        double currPosition = robot.jewelStick.getPosition();
        if (gamepad2.dpad_down) {
            robot.jewelStick.setPosition(Math.min(currPosition + 0.15, JEWEL_STICK_MAX));
        }
        if (gamepad2.dpad_up) {
            robot.jewelStick.setPosition(Math.max(currPosition - 0.15, JEWEL_STICK_MIN));
        }
    }
}
