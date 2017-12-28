package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Bitmap;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

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

    boolean isClamping = false;

    VuforiaLocalizerImplSubclass vuforia; //stores our instance of the Vuforia localization engine


    double INIT_COMPASS_VALUE;

    @Override
    public void runOpMode(){
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.accelSensor = new Accelerometer(hardwareMap);

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

            telemetry.addData("hi", "hello");
            telemetry.addData("Accelerometer", robot.accelSensor.toString());
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

    private void doClamping() {
        if (gamepad1.dpad_up) {
            //make claws clamp onto the object
            robot.clawLeft.setPosition(0.01);
            robot.clawRight.setPosition(0.99);
            isClamping = true;
        }
        if (gamepad1.dpad_down) {
            //make claws release the object
            robot.clawLeft.setPosition(0.99);
            robot.clawRight.setPosition(0.01);
            isClamping = false;
        }
        if (!isClamping) {
            //make claws hang loose if there is nothing to do
            robot.clawLeft.setPosition(0.5);
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

    //random thing for future reference
//    for (VuforiaTrackable beac : beacons) {
//        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getRawPose();
//
//        if (pose != null) {
//            VectorF translation = pose.getTranslation();
//            telemetry.addData(beac.getName() + " - Translation", translation);
//            double radiansToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
//            telemetry.addData(beac.getName() + " - Degrees", radiansToTurn);
//        }
//    }
}
