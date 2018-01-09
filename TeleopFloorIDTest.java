package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.File;

@TeleOp(name="TeleopFloorIDTest", group="FIREWORKS")
public class TeleopFloorIDTest extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareTest robot = new HardwareTest();              // Use a K9'shardware
    ElapsedTime robotTime = new ElapsedTime();

    VuforiaLocalizerImplSubclass vuforia; //stores our instance of the Vuforia localization engine

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        doVuforiaInitialization();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (robotTime.seconds() < 6.5) {
            //wait to let the camera things boot
        }
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            doFloorSensing();
        
            telemetry.addData("hi", "hello");
            telemetry.update();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
    /**
     * Initializes the Vuforia engine
     * @return whether initialization was successful
     */
    public void doVuforiaInitialization() {
        //uncomment this to show what the camera sees
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AVf224j/////AAAAGXlS5fEZWkUuukmNJ278W4N56l4Z/TC6awPG5XTapSLGWsXBBcbc7q+C00X3DfcAs1KmILva7ZKd6OAUyTyZ4fAHK2jrLL56vjoWLOZ1+Gr1ZGya6OYBcQmnbFbUrlGLhnyWtqkIu+RwGApf+LZW18bAaBzo2KOpaZZIaD+UJJ1PqzqtM/v4KH+FXBb4LHN4iHe+q1/gabF8m8Qv+Y2i1407Dre4K/mUp2N+6959a0ZckVqcesMhWtUrljKpie664FXHjYQYPIDQwKiSJfsg12nx4s7rto4ZYmAuTWdcwGZeWHz3gb5rutPgyuG5WiApPnL66MyQNsbA8K1DoK/75pGfY1M2GRzCnzrzenNHLZVt";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = new VuforiaLocalizerImplSubclass(params);
    }
    /**
     * Senses the position of each type of jewel (red or blue)
     * and acts on it. (ALL OTHER ROBOT MOVEMENTS ARE STOPPED)
     */
    private void doFloorSensing() {
        if (vuforia.rgb != null) {
            Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());
            
            //initialize map of floor
            int[] floorHeights = new int[bm.getWidth()];
            int mean = 0;
            
            //find the floor for each column of pixels
            //gray is made by near-equivalent rgb values
            //let's say it's gray if both g and b don't vary from r by over 20
            for (int x = 0; x < bm.getWidth(); x++) {
                for (int y = 0; y < bm.getHeight(); y++) {
                    int color = bm.getPixel(x, y);
                    int r = (color & 0xff0000) >> 16;
                    int g = (color & 0x00ff00) >> 8;
                    int b = color & 0xff;
                    
                    if (Math.abs(r-g) <= 20 && Math.abs(r-b) <= 20) {
                        //found a possible piece of floor
                        floorHeights[x]++;
                    }
                }
                mean += floorHeights[x];
            }
            mean /= floorHeights.length;
          
            //find standard deviation of floor heights
            int std = 0;
            for (int i = 0; i < floorHeights.length; i++) {
                std += (floorHeights[i] - mean) * (floorHeights[i] - mean);
            }
            std = Math.sqrt(std / floorHeights.length);
            
            /*
            * if the edges are on opposite sides of the mean
            *  and they are both 2 std deviations away from the
            * mean, the robot is tilting away from the wall
            */
            int leftHeight = 0;
            int rightHeight = 0;
            for (int i = 0; i < floorHeights.length / 60; i++) {
                leftHeight += floorHeights[i];
                rightHeight += floorHeights[floorHeights.length - i - 1];
            }
            leftHeight /= floorHeights.length / 60;
            rightHeight /= floorHeights.length / 60;
            
            if (Math.abs(leftHeight - mean) > std * 2
                    && Math.abs(rightHeight - mean) > std * 2
                    && ((leftHeight > mean && rightHeight < mean) 
                        || (leftHeight < mean && rightHeight > mean)) {
                //it is tilting away from the wall. Change motor speeds
            }
        }
    }
}
