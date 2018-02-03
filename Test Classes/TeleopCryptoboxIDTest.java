package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.File;

@TeleOp(name="TeleopJewelIDTest", group="FIREWORKS")
public class TeleopJewelIDTest extends LinearOpMode {
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

        //FOR AUTONOMOUS ONLY LOL: push the correct jewel before moving
        doJewelSensing();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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
    private void doJewelSensing() {
        telemetry.addData("Starting", "Jewel Sensing" + robotTime.seconds());
        telemetry.update();
        if (vuforia.rgb != null) {
            Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());

            //After 50 pixels, this multiplier will be 2.69
            final double multiplierInit = 1.02;
            double multiplier = multiplerInit;

            double redSumMultiplied = 0;
            double redSumBase = 0;
            for (int y = 0; y < bm.getHeight(); y++) {
                for (int x = bm.getWidth() / 2; x < bm.getWidth(); x++) {
                    int color = bm.getPixel(x, y);
                    int red = ((color & 0xff0000) >> 16) / 100.0;
                    int blue = (color & 0xff) / 100.0;
                    
                    if (red > 100 && red * 0.5 > blue) {
                        if (multiplier > multiplierInit) {
                            multiplier *= multiplierInit;
                        }
                        else {
                            multipler = multiplierInit;
                        }
                    }
                    else if (red > 100 && blue * 0.5 > red) {
                        if (multiplier > multiplierInit) {
                            multiplier = multiplierInit;
                        }
                        else {
                            multiplier *= multiplierInit;
                        }
                    }
                    
                    //For _multiplier_ variable: The longer the chain of consecutive
                    //red pixels, the more weight we give the red and less the blue
                    redSumMultiplied += red * multiplier;
                    redSumBase += red;
                }
            }

            for (int y = 0; y < bm.getHeight(); y++) {
                for (int x = 0; x < bm.getWidth() / 2; x++) {
                    int color = bm.getPixel(x, y);
                    int red = ((color & 0xff0000) >> 16) / 100.0;
                    int blue = (color & 0xff) / 100.0;
                    
                    if (red * 0.5 > blue) {
                        if (multiplier > multiplierInit) {
                            multiplier *= multiplierInit;
                        }
                        else {
                            multipler = multiplierInit;
                        }
                    }
                    else if (blue * 0.5 > red) {
                        if (multiplier > multiplierInit) {
                            multiplier = multiplierInit;
                        }
                        else {
                            multiplier *= multiplierInit;
                        }
                    }
                    
                    //For _multiplier_ variable: The longer the chain of consecutive
                    //red pixels, the more weight we give the red and less the blue
                    redSumMultiplied -= red * multiplier;
                    redSumBase -= red;
                }
            }

            double certaintyForCrypto = redSumMultiplied - redSumBase;
            double movementStartTime = robotTime.milliseconds();
            if (certaintyForCrypto > 0) {
                //Crypobox is probably present
//                 while (robotTime.milliseconds() < movementStartTime + 2000) {
                    telemetry.addData("Cryptobox", "is present");
                    telemetry.update();
//                 }
            }
            else {
                //Cryptobox is probably not present
//                 while (robotTime.milliseconds() < movementStartTime + 2000) {
                    telemetry.addData("Cryptobox", "is not present");
                    telemetry.update();
//                 }
            }
        }
    }
}
