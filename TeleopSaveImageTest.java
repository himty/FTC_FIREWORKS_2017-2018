package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.media.ImageWriter;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;

@TeleOp(name="TeleopSaveImageTest", group="VuforiaFIREWORKS")
public class TeleopSaveImageTest extends LinearOpMode {
    /*
     *Declare OpMode members.
     */
    HardwareTest    robot               = new HardwareTest();              // Use a K9'shardware
    ElapsedTime     ballDropperTime     = new ElapsedTime(1000); //starting time is high so doesn't mess with timing
    ElapsedTime     bPusherTime         = new ElapsedTime(1000);
    final double MAX_JOYSTICK_VALUE = 1;
    double targetLeftPower;
    double targetRightPower;
    double INIT_COMPASS_VALUE;

    /*
     * Vuforia variables
     */
    OpenGLMatrix lastLocation = null;
//    VuforiaLocalizer vuforia;
    VuforiaLocalizerImplSubclass vuforia; //stores our instance of the Vuforia localization engine
    int cameraMonitorViewId;
    File directory;
    VuforiaTrackables beacons;
    int fileCount = 1;

    Image img;
    FileOutputStream out;

    @Override
    public void runOpMode(){
        doRobotInitialization();

        doVuforiaInitialization();

        //Wait for the driver to start the program
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        //for vumark tracking. Idk why this is here
//        beacons.activate();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.x) {
                doVuforiaLoop();
            }

            doRobotMovementLoop();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    public void doRobotMovementLoop() {
        //            //drive train
//            targetLeftPower = (gamepad1.right_stick_y - 2*gamepad1.right_stick_x);
//            targetRightPower = (gamepad1.right_stick_y + 2 * gamepad1.right_stick_x);
//
//            if (targetLeftPower < 0 && targetRightPower > 0) {
//                targetRightPower = targetRightPower * 0.5;
//            }
//
//            if (gamepad1.right_bumper){
//                targetLeftPower /= 5;
//                targetRightPower /= 5;
//            }
//
////            targetLeftPower = scaleInput(Range.clip(targetLeftPower, -1, 1));
////            targetRightPower = scaleInput(Range.clip(targetLeftPower, -1, 1));
//            robot.leftMotor.setPower(targetLeftPower);
//            robot.leftMotor2.setPower(targetLeftPower);
//            robot.rightMotor.setPower(targetRightPower);
//            robot.rightMotor2.setPower(targetRightPower);
//
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
    }

    /**
     * Initializes the robot
     * @return whether initialization was successful
     */
    public boolean doRobotInitialization() {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
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

        return true;
    }

    /**
     * Initializes the Vuforia engine
     * @return whether initialization was successful
     */
    public boolean doVuforiaInitialization() {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AVf224j/////AAAAGXlS5fEZWkUuukmNJ278W4N56l4Z/TC6awPG5XTapSLGWsXBBcbc7q+C00X3DfcAs1KmILva7ZKd6OAUyTyZ4fAHK2jrLL56vjoWLOZ1+Gr1ZGya6OYBcQmnbFbUrlGLhnyWtqkIu+RwGApf+LZW18bAaBzo2KOpaZZIaD+UJJ1PqzqtM/v4KH+FXBb4LHN4iHe+q1/gabF8m8Qv+Y2i1407Dre4K/mUp2N+6959a0ZckVqcesMhWtUrljKpie664FXHjYQYPIDQwKiSJfsg12nx4s7rto4ZYmAuTWdcwGZeWHz3gb5rutPgyuG5WiApPnL66MyQNsbA8K1DoK/75pGfY1M2GRzCnzrzenNHLZVt";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = new VuforiaLocalizerImplSubclass(params);


        //for looking at the images.. why is this here
//        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);
//        beacons = vuforia.loadTrackablesFromAsset("RelicVuMark");
//        //TODO: beacons.get(0).setName("something");
//        // etc..

        /*
         * Image saving stuff
         */
        directory = new File(Environment.getExternalStorageDirectory().getAbsolutePath()+"/VuforiaImages");
        if(!directory.exists()){
            directory.mkdir();
        }

        return true;
    }

    /**
     * Saves a bitmap from the VuforiaLocalizerImplSubclass
     */
    public void doVuforiaLoop() {
        if (vuforia.rgb != null) {
            Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());

            File learningFile = null;
            while (fileCount < Integer.MAX_VALUE) {
                learningFile = new File(directory.getAbsolutePath() + "/img" + fileCount + ".jpg");
                if (!learningFile.exists()) {
                    telemetry.addData("Saving Image to", Integer.toString(fileCount));
                    telemetry.update();
                    break;
                }
                fileCount++;
            }

            try {
                out = new FileOutputStream(learningFile);
            }
            catch (FileNotFoundException fnfe) {
                telemetry.addData("Error", "Vuforia file not found");
                telemetry.addData("Error", fnfe.toString());
                telemetry.update();
            }

            try {
               bm.compress(Bitmap.CompressFormat.PNG, 100, out);
            }
            catch(Exception e) {
                telemetry.addData("Error", "Vuforia cannot save image");
                telemetry.addData("Error", e.toString());
                telemetry.update();
            }
            finally {
                try {
                    if (out != null) {
                        out.close();
                    }
                }
                catch (IOException e) {
                    telemetry.addData("Error", "Vuforia cannot save image. Closed out file stream.");
                    telemetry.addData("Error", e.toString());
                    telemetry.update();
                }
            }
        }
        telemetry.update();
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
