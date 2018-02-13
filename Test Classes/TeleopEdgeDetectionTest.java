package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Picture;
import android.graphics.Point;
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
import java.util.ArrayList;
import java.util.List;

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
//            if ((int)robotTime.seconds() % 10 == 0) {
                doSobelEdgeDetectionAndSave();
//            }

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

    public void doSobelEdgeDetectionAndSave() {
        if (vuforia.rgb != null) {
            telemetry.addData("Edge Detection", "Starting");
            telemetry.update();

            Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());
            //resize this bitmap
            bm = getRescaledBitmap(bm, 200, false);

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

            Bitmap bmEdges = Bitmap.createScaledBitmap(bm, bm.getWidth(), bm.getHeight(), false);
            telemetry.addData("Edge Detection", "Created new bitmap");
            telemetry.update();

            for (int y = 1; y < bm.getHeight() - 2; y++) {
                for (int x = 1; x < bm.getWidth() - 2; x++) {
                    telemetry.addData("Edge Detection", "x-" + x + " y-" + y);
                    telemetry.update();

                    // get 3-by-3 array of colors in neighborhood
                    int[][] gray = new int[3][3];
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            //Calculate intensity
                            //This is the average of the rgb values
                            int color = bm.getPixel(x+i, y+j);
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
                    bmEdges.setPixel(x, y, Color.argb(1, magnitude, magnitude, magnitude));
                }
            }

            telemetry.addData("Edge Detection", "Finished applying filters. Starting line detection");
            telemetry.update();

            //Detect collinear points on vertical lines. Get x-positions of those vertical lines
            ArrayList<Line> lines = getLinesFromBitmap(bm);

            telemetry.addData("Edge Detection", "Got lines from Bitmap");
            telemetry.update();

            //FOR DEBUGGING: overlay these lines on the original bitmap and save it
            Bitmap bmLines = Bitmap.createScaledBitmap(bm, bm.getWidth(), bm.getHeight(), false);
            BresenhamLineMaker b = new BresenhamLineMaker();

            for (Line l : lines) {
                // (y-a) = m(x-b)
                // y = mx - bm + a
                Point pt = l.getPoint();

                List<Point> pixelLine = b.findLine(bmLines,
                        0,
                        (int)(pt.x * l.getSlope() + pt.y),
                        bmLines.getWidth(),
                        (int)(l.getSlope()*bmLines.getWidth() - pt.x * l.getSlope() + pt.y));

                b.plot(bm, pixelLine);
            }

            telemetry.addData("Edge Detection", "Plotted lines in Bitmap");
            telemetry.update();

            //TODO: find lines that point in the same direction

            //save image
//            saveBitmaps(new Bitmap[]{bm, bmEdges}, "EdgeDetectionTest", new String[]{"raw", "filtered"});
            saveBitmaps(new Bitmap[]{bm, bmEdges, bmLines}, "EdgeDetectionTest", new String[]{"raw", "filtered", "line"});

            telemetry.addData("Edge Detection", "Finished");
            telemetry.update();
        }
        else {
            telemetry.addData("Edge Detection", "No image found");
            telemetry.update();
        }
    }

    public void doGeneralEdgeDetectionAndSave() {
        if (vuforia.rgb != null) {
            telemetry.addData("Edge Detection", "Starting");
            telemetry.update();

            Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());
            //resize this bitmap
            bm = getRescaledBitmap(bm, 200, true);

            telemetry.addData("Edge Detection", "Retrieved bitmap");
            telemetry.update();

            int[][] filter1 = {
                    {-1, 0, 1},
                    {0,  0, 0},
                    {1, 0, -1}
            };

            Bitmap bmEdges = Bitmap.createScaledBitmap(bm, bm.getWidth(), bm.getHeight(), true);
            telemetry.addData("Edge Detection", "Created new bitmap");
            telemetry.update();

            for (int y = 1; y < bmEdges.getHeight() - 2; y++) {
                for (int x = 1; x < bmEdges.getWidth() - 2; x++) {
                    telemetry.addData("Edge Detection", "x-" + x + " y-" + y);
                    telemetry.update();

                    // get 3-by-3 array of colors in neighborhood
                    int[][] gray = new int[3][3];
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            //Calculate intensity
                            //This is the average of the rgb values
                            int color = bm.getPixel(x+i, y+j);
                            gray[i][j] = ((color >> 8) & 0xFF + (color >> 16) & 0xFF + (color >> 24) & 0xFF) / 3;
                        }
                    }

                    // apply filter
                    int gray1 = 0, gray2 = 0;
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            gray1 += gray[i][j] * filter1[i][j];
                        }
                    }
                    bmEdges.setPixel(x, y, Color.argb(1, gray1, gray1, gray1));
                }
            }

            telemetry.addData("Edge Detection", "Finished applying filters. Starting Hough Transform");
            telemetry.update();

            //Hough Transform
            HoughTransform houghTransform = new HoughTransform(bmEdges);
            Bitmap bmHough = houghTransform.getOutputBitmap();

            telemetry.addData("Edge Detection", "Got Hough Transform");
            telemetry.update();
//
//            //Detect collinear points on vertical lines. Get x-positions of those vertical lines
//            ArrayList<Line> lines = getLinesFromBitmap(bm);
//
//            telemetry.addData("Edge Detection", "Got lines from Bitmap");
//            telemetry.update();
//
//            //FOR DEBUGGING: overlay these lines on the original bitmap and save it
//            Bitmap bmLines = Bitmap.createScaledBitmap(bm, bm.getWidth(), bm.getHeight(), true);
//            BresenhamLineMaker b = new BresenhamLineMaker();
//
//            for (Line l : lines) {
//                // (y-a) = m(x-b)
//                // y = mx - bm + a
//                Point pt = l.getPoint();
//
//                List<Point> pixelLine = b.findLine(bmLines,
//                        0,
//                        (int)(pt.x * l.getSlope() + pt.y),
//                        bmLines.getWidth(),
//                        (int)(l.getSlope()*bmLines.getWidth() - pt.x * l.getSlope() + pt.y));
//
//                b.plot(bm, pixelLine);
//            }
//
//            telemetry.addData("Edge Detection", "Plotted lines in Bitmap");
//            telemetry.update();

            //TODO: find lines that point in the same direction

            //save image
            saveBitmaps(new Bitmap[]{bm, bmEdges, bmHough}, "EdgeDetectionTest", new String[]{"raw", "filtered", "hough"});
//            saveBitmaps(new Bitmap[]{bm, bmEdges, bmLines}, "EdgeDetectionTest", new String[]{"raw", "filtered", "line"});

            telemetry.addData("Edge Detection", "Finished");
            telemetry.update();
        }
        else {
            telemetry.addData("Edge Detection", "No image found");
            telemetry.update();
        }
    }

    /**
     * Returns all lines with at least 4 collinear points
     * from the image. (Line is a user-created class)
     *
     * Uses the sorting strategy described in
     * http://www.cs.princeton.edu/courses/archive/spring03/cs226/assignments/lines.html
     *
     * @param bm the Bitmap processed in doEdgeDetection
     * @return positions of vertical lines
     */
    public ArrayList<Line> getLinesFromBitmap(Bitmap bm) {
        ArrayList<Line> lineList = new ArrayList<Line>();

        //find all black points in the image
        ArrayList<Point> pointList = new ArrayList<Point>();
        for (int y = 0; y < bm.getHeight(); y++) {
            for (int x = 0; x < bm.getWidth(); x++) {
                //if intensity is less than 20 (black)
                if (((bm.getPixel(x,y) >> 8) & 0xFF) < 100) {
                    pointList.add(new Point(x, y));
                }
            }
        }

        Bitmap bmPlain = Bitmap.createBitmap(bm.getWidth(), bm.getHeight(), Bitmap.Config.RGB_565);
        Bitmap bmPoints = Bitmap.createBitmap(bm.getWidth(), bm.getHeight(), Bitmap.Config.RGB_565);

        for (int y = 0; y < bmPoints.getHeight(); y++) {
            for (int x = 0; x < bmPoints.getWidth(); x++) {
                bmPlain.setPixel(x, y, Color.WHITE);
                bmPoints.setPixel(x, y, Color.WHITE);
            }
        }

        for (Point p : pointList) {
            bmPoints.setPixel(p.x, p.y, Color.BLACK);
        }

        saveBitmaps(new Bitmap[]{bmPlain, bmPoints, bm}, "Other bitmap test", new String[]{"plain","points","raw"});

//
//
//
//
//        telemetry.addData("Edge Detection", "Got black pixels");
//        telemetry.update();
//
//        //For each point, calculate the angle all other points make
//        //with this point, sort the angles, and save collinear points
//        //to
//        for (Point p1 : pointList) {
//            //find angles
//            double[] angles = new double[pointList.size()];
//            for (int i = 0; i < pointList.size(); i++) {
//                Point p2 = pointList.get(i);
//                if (p1 != p2) {
//                    double angle = Math.toDegrees(Math.atan2(p2.y - p1.y, p2.x - p1.x));
//
//                    if (angle < 0) {
//                        angle += 360;
//                    }
//
//                    angles[i] = angle;
//                    i++;
//                }
//            }
//
//            telemetry.addData("Edge Detection", "Got angles");
//            telemetry.update();
//
//            //sort angles
//            Quicksort qs = new Quicksort();
//            qs.sort(angles);
//            angles = qs.getSortedArray();
//
//            telemetry.addData("Edge Detection", "Sorted angles");
//
//            //If 3 points have approximately equal angles with
//            //p, these points, together with p, are collinear.
//
//            //points can be this many degrees apart and still be
//            //considered "collinear"
//            double tolerance = 5;
//            int i = 0;
//            while (i < angles.length - 2) {
//                telemetry.addData("Collinear Detection", i + " out of " + (angles.length - 2));
//                telemetry.update();
//                if (angles[i + 2] - angles[i] < tolerance) {
//                    //these 3 points and our original point
//                    //are collinear
//
//                    lineList.add(new Line(p1, Math.tan(angles[i+1])));
//                    i += 3;
//                }
//                else {
//                    i++;
//                }
//            }
//        }
//
        return lineList;
    }

    public Bitmap getRescaledBitmap(Bitmap realImage, int widthSize,
                                   boolean filter) {
        float ratio = (float) widthSize / realImage.getWidth();
        int width = widthSize;
        int height = Math.round((float) ratio * realImage.getHeight());

        Bitmap newBitmap = Bitmap.createScaledBitmap(realImage, width,
                height, filter);
        return newBitmap;
    }

    public void saveBitmaps(Bitmap[] bms, String folderName, String[] fileNames) {
        telemetry.addData("Edge Detection", "Saving image...");
        telemetry.update();

        if (fileNames.length != bms.length) {
            telemetry.addData("Error", "Cannot save files because # Bitmaps != # file names");
            telemetry.update();
            return;
        }

        File directory = new File(Environment.getExternalStorageDirectory().getAbsolutePath()
                +"/"+folderName);
        if(!directory.exists()){
            directory.mkdir();
        }

        File learningFile = null;
        while (fileCount < Integer.MAX_VALUE) {
            learningFile = new File(directory.getAbsolutePath() + "/img" + fileCount+"_"+fileNames[0]+".jpg");
            if (!learningFile.exists()) {
                telemetry.addData("Saving Image to", Integer.toString(fileCount));
                telemetry.update();
                break;
            }
            fileCount++;
        }

        for (int i = 0; i < bms.length; i++) {
            Bitmap bm = bms[i];
            String fileName = fileNames[i];

            learningFile = new File(directory.getAbsolutePath() + "/img" + fileCount+"_"+fileName+".jpg");

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
//            telemetry.addData("Edge Detection", "Image" + fileCount+" saved");
//            telemetry.update();
        }
    }
}
