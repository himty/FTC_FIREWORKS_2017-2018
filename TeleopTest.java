package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleopTest", group="FIREWORKS")
public class TeleopTest extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareTest    robot               = new HardwareTest();              // Use a K9'shardware
    ElapsedTime     ballDropperTime     = new ElapsedTime(1000); //starting time is high so doesn't mess with timing
    ElapsedTime     bPusherTime         = new ElapsedTime(1000);

    final double MAX_JOYSTICK_VALUE = 1;

    double targetLeftPower;
    double targetRightPower;

    double INIT_COMPASS_VALUE;

    @Override
    public void runOpMode(){
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
//
//        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //calibrate compass sensor
//        robot.compSensor.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
//        bPusherTime.reset();
//        while (bPusherTime.seconds() < 4){
//            ;
//        }
//        if (robot.compSensor.calibrationFailed()){
//            telemetry.addData("Say", "Compass Calibration Failed");    //
//            telemetry.update();
//        }
//        robot.compSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
//        INIT_COMPASS_VALUE = robot.compSensor.getDirection();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //drive train
            targetLeftPower = (gamepad1.right_stick_y - 2*gamepad1.right_stick_x);
            targetRightPower = (gamepad1.right_stick_y + 2 * gamepad1.right_stick_x);

            if (targetLeftPower < 0 && targetRightPower > 0) {
                targetRightPower = targetRightPower * 0.5;
            }

            if (gamepad1.right_bumper){
                targetLeftPower /= 5;
                targetRightPower /= 5;
            }

//            targetLeftPower = scaleInput(Range.clip(targetLeftPower, -1, 1));
//            targetRightPower = scaleInput(Range.clip(targetLeftPower, -1, 1));
            robot.frontleftMotor.setPower(targetLeftPower);
            robot.backleftMotor.setPower(targetLeftPower);
            robot.frontrightMotor.setPower(targetRightPower);
            robot.backrightMotor.setPower(targetRightPower);

            //linear slide
            robot.linearSlide.setPower(-1 * gamepad2.right_stick_y);
//
//            //ball holder movement
//            robot.ballHolder.setPower(gamepad2.left_stick_y);
//
//            //beacon pusher movement
//            if (gamepad1.dpad_up){
//                robot.beaconPusher.setPower(0.5);
//            } else if (gamepad1.dpad_down) {
//                robot.beaconPusher.setPower(-0.5);
//            } else {
//                robot.beaconPusher.setPower(0);
//            }
//
//            //popper
////            if (gamepad2.dpad_up){
////                robot.popper.setPower(1);
////            } else if (gamepad2.dpad_down) {
////                robot.popper.setPower(-1);
////            } else {
////                robot.popper.setPower(0);
////            }
////
//            // Send telemetry message to signify robot running;
////            telemetry.addData("Ball Dropper Position",   "%.2f", robot.ballDropper.getPosition());
////            telemetry.addData("true/false", Double.isNaN(robot.ballDropper.getPosition()));
//            telemetry.addData("left",  "%.2f", robot.leftMotor.getPower());
//            telemetry.addData("right", "%.2f", robot.rightMotor.getPower());
//
////            telemetry.addData("Light", "%.2f %.2f %.2f", robot.lightSensor.getRawLightDetected(), robot.lightSensor.getLightDetected(), robot.lightSensor.getRawLightDetectedMax());
////            telemetry.addData("Compass", "%.2f", robot.compSensor.getDirection()-INIT_COMPASS_VALUE);

            telemetry.addData("hi", "hello");
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) ((dVal/MAX_JOYSTICK_VALUE) * 16.0); //number is now <= 1

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
