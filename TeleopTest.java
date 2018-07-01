package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleopTest", group="FIREWORKS")
public class TeleopTest extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareTest    robot               = new HardwareTest();              // Use a K9'shardware

    //Max is the bottom position for the jewel stick
    //Min is the upper position for the jewel stick
    //To make the stick move farther from the robot, raise this value
//    final double JEWEL_STICK_MAX = 0.56;
//    final double JEWEL_STICK_MIN = 0.07;

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
            doLifter();
            doJewelHitter();
            doRampCollector();

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
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;

        //Turning is configured on separate joystick
        double rightX = gamepad1.right_stick_x * 0.5;
        double v1 = r * Math.cos(robotAngle) - rightX;
        double v2 = r * Math.sin(robotAngle) + rightX;
        double v3 = r * Math.sin(robotAngle) - rightX;
        double v4 = r * Math.cos(robotAngle) + rightX;

        if (!gamepad1.right_bumper){
            v1 *= 0.6;
            v2 *= 0.6;
            v3 *= 0.6;
            v4 *= 0.6;
        }

        //Set powers
        robot.frontleftMotor.setPower(v1);
        robot.frontrightMotor.setPower(v2);
        robot.backleftMotor.setPower(v3);
        robot.backrightMotor.setPower(v4);
    }

    private void doLifter() {
        robot.lifter.setPower(gamepad2.right_stick_y * 0.7);
    }

    private void doRampCollector() {
            robot.rampLeft.setPower(gamepad2.left_stick_y);
            robot.rampRight.setPower(gamepad2.left_stick_y);
    }

    private void doJewelHitter() {
        if (gamepad2.dpad_down) {
            //make jewelstick go down
            robot.jewelStick.setPower(-0.5);

        } else if (gamepad2.dpad_up) {
            //make jewelstick go up
            robot.jewelStick.setPower(0.5);
        }
        else {
            //make jewelstick stay there if there's nothing to do
            robot.jewelStick.setPower(0);
        }
    }
}
