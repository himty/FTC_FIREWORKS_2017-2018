package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.media.ImageWriter;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
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
    int VUFORIA_FRAME_QUEUE_CAPACITY = 10;
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia; //stores our instance of the Vuforia localization engine
    int cameraMonitorViewId;
    File directory;

    @Override
    public void runOpMode(){
        doRobotInitialization();

        doVuforiaInitialization();

        //Wait for the driver to start the program
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            doImageSavingLoop();
//
            doRobotMovementLoop();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    public void doImageSavingLoop() {
        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = this.vuforia.getFrameQueue();
        // if new frames are available
        telemetry.addData("Number new frames: ", frameQueue.size()); //always 10 (max)
        telemetry.update();
        while(!frameQueue.isEmpty()) {
            try {
                VuforiaLocalizer.CloseableFrame f = frameQueue.take();
                long numImages = Math.min(f.getNumImages(), Integer.MAX_VALUE);
                for (int i = 0; i < numImages; i++) {
                    Image img = f.getImage(i);
                    FileOutputStream out;

                    int count = 1;
                    File learningFile;
                    while (count < Integer.MAX_VALUE) {
                        learningFile = new File(directory.getAbsolutePath() + "/img" + count + ".jpg");
                        if (!learningFile.exists()) {
                            break;
                        }
                        count++;
                    }

                    ByteBuffer buffer = img.getPixels();
                    byte[] bytes = new byte[buffer.capacity()];
                    buffer.get(bytes);
                    Bitmap bitmapImage = BitmapFactory.decodeByteArray(bytes, 0, bytes.length, null);

                    try {
                        out = new FileOutputStream(directory.getAbsolutePath() + "/img" + count + ".jpg");

//                        bitmapImage.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
//                        // PNG is a lossless format, the compression factor (100) is ignored
                        out.write(bytes, 0, bytes.length);

                        out.flush(); // Not really required
                        out.close(); // do not forget to close the stream
                    }
                    catch(FileNotFoundException fnfe) {
                        telemetry.addData("Error", "Vuforia image saving location not found");
                        telemetry.addData("Error", fnfe.toString());
                        telemetry.update();
                    }
                    catch(IOException ioe) {
                        telemetry.addData("Error", "Vuforia image saving cannot be done.");
                        telemetry.addData("Error", ioe.toString());
                        telemetry.update();
                    }
                    telemetry.addData("Hi", "8");
                    telemetry.update();
                }
            }
            catch (InterruptedException i) {
                telemetry.addData("Error", "Retrieval of frame from Vuforia frame queue inturrupted.");
            }
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
        //To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
        //If no camera monitor is desired, use the parameterless constructor instead (commented out below).
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
//         VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * Set the license key for Vuforia so that you can use the Vuforia library
         */
        parameters.vuforiaLicenseKey = "AVf224j/////AAAAGXlS5fEZWkUuukmNJ278W4N56l4Z/TC6awPG5XTapSLGWsXBBcbc7q+C00X3DfcAs1KmILva7ZKd6OAUyTyZ4fAHK2jrLL56vjoWLOZ1+Gr1ZGya6OYBcQmnbFbUrlGLhnyWtqkIu+RwGApf+LZW18bAaBzo2KOpaZZIaD+UJJ1PqzqtM/v4KH+FXBb4LHN4iHe+q1/gabF8m8Qv+Y2i1407Dre4K/mUp2N+6959a0ZckVqcesMhWtUrljKpie664FXHjYQYPIDQwKiSJfsg12nx4s7rto4ZYmAuTWdcwGZeWHz3gb5rutPgyuG5WiApPnL66MyQNsbA8K1DoK/75pGfY1M2GRzCnzrzenNHLZVt";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Not necessary if we're not tracking the relic image
//        /**
//         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
//         * in this data set: all three of the VuMarks in the game were created from this one template,
//         * but differ in their instance id information.
//         * @see VuMarkInstanceId
//         */
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//        relicTrackables.activate();

        /*
         * Image saving stuff
         */
        //tell vuforia to set aside 10 camera frames for us to analyze
        this.vuforia.setFrameQueueCapacity(VUFORIA_FRAME_QUEUE_CAPACITY);
        directory = new File(Environment.getExternalStorageDirectory().getAbsolutePath()+"/VuforiaImages");
        if(!directory.exists()){
            directory.mkdir();
        }

        return true;
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
