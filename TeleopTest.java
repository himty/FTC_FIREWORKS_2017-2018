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

    double pid_errorPrior = 0;
    double pid_integral = 0;
    double pid_derivative;
    ElapsedTime pid_timer = new ElapsedTime();
    double kp = 0.5;
    double ki = 0;
    double kd = 0;

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
        INIT_COMPASS_VALUE = robot.compSensor.getDirection();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        pid_timer.reset();
        while (opModeIsActive()) {
            doDriveTrain();
            doClampingThingy();
            doLinearSlide();

//            // Send telemetry message to signify robot running;
//            telemetry.addData("Ball Dropper Position",   "%.2f", robot.ballDropper.getPosition());
//            telemetry.addData("true/false", Double.isNaN(robot.ballDropper.getPosition()));
//            telemetry.addData("left",  "%.2f", robot.leftMotor.getPower());
//            telemetry.addData("right", "%.2f", robot.rightMotor.getPower());
//
//            telemetry.addData("Light", "%.2f %.2f %.2f", robot.lightSensor.getRawLightDetected(), robot.lightSensor.getLightDetected(), robot.lightSensor.getRawLightDetectedMax());
//            telemetry.addData("Compass", "%.2f", robot.compSensor.getDirection()-INIT_COMPASS_VALUE);

            telemetry.addData("hi", "hello");
            telemetry.addData("compass", robot.compSensor.getDirection());
            telemetry.addData("servoleft", robot.clawLeft.getPosition());
            telemetry.addData("servoright", robot.clawRight.getPosition());
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    /**
     * Makes the robot's drive train move
     */
    private void doDriveTrain() {
        //calculate motor powers
        targetLeftPower = (gamepad1.right_stick_y - 2*gamepad1.right_stick_x);
        targetRightPower = (gamepad1.right_stick_y + 2 * gamepad1.right_stick_x);

        if (targetLeftPower < 0 && targetRightPower > 0) {
            targetRightPower = targetRightPower * 0.5;
        }

        if (gamepad1.right_bumper){
            targetLeftPower /= 5;
            targetRightPower /= 5;
        }

        //set powers
//            targetLeftPower = scaleInput(Range.clip(targetLeftPower, -1, 1));
//            targetRightPower = scaleInput(Range.clip(targetLeftPower, -1, 1));
        robot.frontleftMotor.setPower(targetLeftPower);
        robot.backleftMotor.setPower(targetLeftPower);
        robot.frontrightMotor.setPower(targetRightPower);
        robot.backrightMotor.setPower(targetRightPower);
    }

    private void doClampingThingy() {
        //different behaviors for the servos because they are different types of servos
        if (gamepad1.dpad_up) {
            robot.clawLeft.setPosition(robot.clawLeft.getPosition()+0.01);
        }
        else if(gamepad1.dpad_down) {
            robot.clawLeft.setPosition(robot.clawLeft.getPosition()-0.01);
        }

        if (gamepad1.y) {
            robot.clawRight.setPosition(0.75);
        }
        else if (gamepad1.a) {
            robot.clawRight.setPosition(0.25);
        }
        else {
            robot.clawRight.setPosition(0.5);
        }
    }

    private void doLinearSlide() {
        robot.linearSlide.setPower(-1 * gamepad2.right_stick_y);
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

    /**
     * Returns motor powers for drive train after
     * using PID control algorithm
     * @param desiredDirection
     * @param errorPrev
     * @return [motorLeft power, motorRight power]
     */
    double[] getPIDPowers(double desiredDirection, double errorPrev) {
        double[] resultPowers = new double[2];
        double relativeRobotDir = robot.compSensor.getDirection() - INIT_COMPASS_VALUE;
        //calculate PID stuff
        double iterationTime = pid_timer.milliseconds();
        double error = desiredDirection - relativeRobotDir;
        pid_integral += error * iterationTime;
        pid_derivative = (error - pid_errorPrior) / iterationTime;

        //can also add a bias to this
        double targetDir = kp*error + ki*pid_integral + kd*pid_derivative;
        pid_errorPrior = error;

        resultPowers[0] = robot.frontrightMotor.getPower() + 0.5 * (relativeRobotDir - targetDir);
        resultPowers[1] = robot.frontleftMotor.getPower() - 0.5 * (relativeRobotDir - targetDir);

        pid_timer.reset();

        return resultPowers;
    }
}
