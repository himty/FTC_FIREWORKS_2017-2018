package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Picture;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

@TeleOp(name="TeleopEdgeDetectionTest", group="VuforiaFIREWORKS")
public class TeleopEdgeDetectionTest extends LinearOpMode {
    /*
     *Declare OpMode members.
     */
    HardwareTest    robot               = new HardwareTest();
    ElapsedTime     robotTime           = new ElapsedTime();

    /*
     * Vuforia variables
     */
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizerImplSubclass vuforia; //stores our instance of the Vuforia localization engine
    int fileCount = 1;

    Image img;
    FileOutputStream out;

    @Override
    public void runOpMode(){
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        doVuforiaInitialization();

        //Wait for the driver to start the program
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if ((int)robotTime.seconds() % 10 == 0) {
                doEdgeDetectionAndSave();
            }

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    /**
     * Initializes the Vuforia engine
     * @return whether initialization was successful
     */
    public void doVuforiaInitialization() {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AVf224j/////AAAAGXlS5fEZWkUuukmNJ278W4N56l4Z/TC6awPG5XTapSLGWsXBBcbc7q+C00X3DfcAs1KmILva7ZKd6OAUyTyZ4fAHK2jrLL56vjoWLOZ1+Gr1ZGya6OYBcQmnbFbUrlGLhnyWtqkIu+RwGApf+LZW18bAaBzo2KOpaZZIaD+UJJ1PqzqtM/v4KH+FXBb4LHN4iHe+q1/gabF8m8Qv+Y2i1407Dre4K/mUp2N+6959a0ZckVqcesMhWtUrljKpie664FXHjYQYPIDQwKiSJfsg12nx4s7rto4ZYmAuTWdcwGZeWHz3gb5rutPgyuG5WiApPnL66MyQNsbA8K1DoK/75pGfY1M2GRzCnzrzenNHLZVt";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = new VuforiaLocalizerImplSubclass(params);
    }

    // truncate color component to be between 0 and 255
    public static int truncate(int a) {
        if      (a <   0) return 0;
        else if (a > 255) return 255;
        else              return a;
    }

    public void doEdgeDetectionAndSave() {
        if (vuforia.rgb != null) {
            telemetry.addData("Edge Detection", "Starting");
            telemetry.update();

            Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());

            telemetry.addData("Edge Detection", "Retrieved bitmap");
            telemetry.update();

            int[][] filter1 = {
                    {-1, 0, 1},
                    {-2, 0, 2},
                    {-1, 0, 1}
            };

            int[][] filter2 = {
                    {1, 2, 1},
                    {0, 0, 0},
                    {-1, -2, -1}
            };

            Bitmap bmNew = getRescaledBitmap(bm, 200, true);
//                    = Bitmap.createBitmap(widthNew, heightNew, Bitmap.Config.ARGB_8888);
            int scalingWidth = bm.getWidth() / bmNew.getWidth();
            int scalingHeight = bm.getHeight() / bmNew.getHeight();

            telemetry.addData("Edge Detection", "Created new bitmap");
            telemetry.update();

            for (int y = 1; y < bmNew.getHeight(); y++) {
                for (int x = 1; x < bmNew.getWidth(); x++) {
                    telemetry.addData("Edge Detection", "x-" + x + " y-" + y);
                    telemetry.update();

                    // get 3-by-3 array of colors in neighborhood
                    int[][] gray = new int[3][3];
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            //Calculate intensity
                            //This is the average of the rgb values
                            int color = bm.getPixel(x*scalingWidth+i, y*scalingHeight+j);
                            gray[i][j] = ((color >> 8) & 0xFF + (color >> 16) & 0xFF + (color >> 24) & 0xFF) / 3;
                        }
                    }

                    // apply filter
                    int gray1 = 0, gray2 = 0;
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            gray1 += gray[i][j] * filter1[i][j];
                            gray2 += gray[i][j] * filter2[i][j];
                        }
                    }
                    // int magnitude = 255 - truncate(Math.abs(gray1) + Math.abs(gray2));
                    int magnitude = 255 - truncate((int) Math.sqrt(gray1 * gray1 + gray2 * gray2));
                    bmNew.setPixel(x, y, Color.argb(1, magnitude, magnitude, magnitude));
                }
            }

            telemetry.addData("Edge Detection", "Finished applying filters. Starting Save");
            telemetry.update();

            //save image
            saveBitmap(bm, "EdgeDetectionTest", "raw");
            saveBitmap(bmNew, "EdgeDetectionTest", "filtered");

            telemetry.addData("Edge Detection", "Finished");
            telemetry.update();
        }
        else {
            telemetry.addData("Edge Detection", "No image found");
        }
    }

    public static Bitmap getRescaledBitmap(Bitmap realImage, float maxImageSize,
                                   boolean filter) {
        float ratio = Math.min(
                (float) maxImageSize / realImage.getWidth(),
                (float) maxImageSize / realImage.getHeight());
        int width = Math.round((float) ratio * realImage.getWidth());
        int height = Math.round((float) ratio * realImage.getHeight());

        Bitmap newBitmap = Bitmap.createScaledBitmap(realImage, width,
                height, filter);
        return newBitmap;
    }

    public void saveBitmap(Bitmap bm, String folderName) {
        saveBitmap(bm, folderName, "");
    }

    public void saveBitmap(Bitmap bm, String folderName, String fileName) {
        telemetry.addData("Edge Detection", "Saving image...");
        telemetry.update();

        File directory = new File(Environment.getExternalStorageDirectory().getAbsolutePath()
                +"/"+folderName);
        if(!directory.exists()){
            directory.mkdir();
        }

        File learningFile = null;
        while (fileCount < Integer.MAX_VALUE) {
            learningFile = new File(directory.getAbsolutePath() + "/img" + fileCount+"_"+fileName+".jpg");
            if (!learningFile.exists()) {
                telemetry.addData("Saving Image to", Integer.toString(fileCount) + "_" + fileName);
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
        telemetry.addData("Edge Detection", "Image" + fileCount+" saved");
        telemetry.update();
    }

}
