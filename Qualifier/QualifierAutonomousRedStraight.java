package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;

import android.graphics.Bitmap;

import static com.sun.tools.javac.util.Constants.format;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTest;

@Autonomous
public class AutonomousRedStraight extends LinearOpMode {
    HardwareTest robot = new HardwareTest();
    private ElapsedTime runtime = new ElapsedTime();
    
    //Encodes the robot's starting position to guide the
    //robot throughout the autonomous program
    final String TEAM_COLOR = "RED"; //can be RED or BLUE
    final String TEAM_POSITION = "STRAIGHT"; //can be CURVED or STRAIGHT

    //Max is the bottom position for the jewel stick
    //Min is the upper position for the jewel stick
    //To make the stick move farther from the robot, raise this value
    final double JEWEL_STICK_MAX = 0.56;
    final double JEWEL_STICK_MIN = 0.07;
    
    //Multiplier to make the forward drivetrain
    //movement direction equal a positive number
    final int DIRECTION = -1;

    /*
     * Vuforia variables
     */
    RelicRecoveryVuMark currentVuMark = RelicRecoveryVuMark.UNKNOWN;
    VuforiaLocalizerImplSubclass vuforia; //stores our instance of the Vuforia localization engine
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;


    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        initVuforia();
        //robot.accelSensor = new Accelerometer(hardwareMap);

        initVuMark();

        robot.frontleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Wait for the camera to boot
        //This avoids getting null images
        while (runtime.seconds() < 6.5) {
            idle();
        }

        relicTrackables.activate();
        
        telemetry.addData("Status", "Running");
        telemetry.update();

        //Clamp onto the block
        robot.clawLeft.setPosition(0.2);
        robot.clawRight.setPosition(0.7);
        runtime.reset();
        while (runtime.seconds() < 1) {
            idle();
        }

        //Lift the block so that it doesn't scrape the floor
        robot.linearSlide.setPower(1);
        runtime.reset();
        while (runtime.seconds() < 4) {
            idle();
        }
        robot.linearSlide.setPower(0);

        //Sense the placement of the jewels,
        //knock off the correct jewel,
        //move to the pictograph, and sense it
        doJewelSensing();

        /*
         *Move to the cryptobox.
         *The current if/else structure allows for different robot ending positions
         *when programming the robot to place the block in a certain column
        */
        ///BLUE + STRAIGHT
        if (TEAM_COLOR.equals("BLUE") && TEAM_POSITION.equals("STRAIGHT")) {
            if (currentVuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("VuMark", "Left");
                telemetry.update();
                move(0.3, 0.3, 2.4);
            }
            else if (currentVuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("VuMark", "RIGHT");
                telemetry.update();
                move(0.3, 0.3, 2.4);
            }
            else {
                telemetry.addData("VuMark", "Center");
                telemetry.update();
                move(0.3, 0.3, 2.4);// move based on unknown or vumark center
            }
        }
        
        //BLUE + CURVED
        else if (TEAM_COLOR.equals("BLUE") && TEAM_POSITION.equals("CURVED")) {
            if (currentVuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("VuMark", "Left");
                telemetry.update();
                move(0.3, 0.3, 1.1);
                move(0.3, -0.3, 1.3);
                move(0.3, 0.3, 0.4);
            }
            else if (currentVuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("VuMark", "RIGHT");
                telemetry.update();
                move(0.3, 0.3, 1.1);
                move(0.3, -0.3, 1.3);
                move(0.3, 0.3, 0.4);
            }
            else {
                telemetry.addData("VuMark", "Center");
                telemetry.update();
                move(0.3, 0.3, 1.1);
                move(0.3, -0.3, 1.3);
                move(0.3, 0.3, 0.4);
            }
        }
        
        //RED + STRAIGHT
        else if (TEAM_COLOR.equals("RED") && TEAM_POSITION.equals("STRAIGHT")) {
            if (currentVuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("VuMark", "Left");
                telemetry.update();
                move(-0.3, -0.3, 2.4);
            }
            else if (currentVuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("VuMark", "RIGHT");
                telemetry.update();
                move(-0.3, -0.3, 2.4);
            }
            else {
                telemetry.addData("VuMark", "Center");
                telemetry.update();
                move(-0.3, -0.3, 2.4);// move based on unknown or vumark center
            }
        }
        
        //RED + CURVED
        else if (TEAM_COLOR.equals("RED") && TEAM_POSITION.equals("CURVED")) {
            if (currentVuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("VuMark", "Left");
                telemetry.update();
                move(-0.3, -0.3, 1.1);
                move(0.3, -0.3, 1.3);
                move(0.3, 0.3, 0.4);
            }
            else if (currentVuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("VuMark", "RIGHT");
                telemetry.update();
                move(-0.3, -0.3, 1.1);
                move(0.3, -0.3, 1.3);
                move(0.3, 0.3, 0.4);
            }
            else {
                telemetry.addData("VuMark", "Center");
                telemetry.update();
                move(-0.3, -0.3, 1.1);
                move(0.3, -0.3, 1.3);
                move(0.3, 0.3, 0.4);
            }
        }

        //Lower the block, resetting the robot before
        //tele-op mode.
        robot.linearSlide.setPower(-1);
        runtime.reset();
        while (runtime.seconds() < 4) {
            idle();
        }
        robot.linearSlide.setPower(0);
    }

    /**
     * Senses the position of each type of jewel (red or blue)
     * and acts on it.
     * 
     * @precondition robot is centered on the jewel stand
     * @postcondition robot has pushed off the correct jewel and is in front of the pictograph
     */
    private void doJewelSensing() {
        telemetry.addData("Starting", "Jewel Sensing" + runtime.seconds());
        telemetry.update();
        
        if (vuforia.rgb != null) {
            //Get bitmap through Vuforia
            Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());
            
            //Total the "amount" of red and blue on the left half of the screen
            double redAvgLeft = 0;
            double blueAvgLeft = 0;
            for (int y = 0; y < bm.getHeight(); y++) {
                for (int x = bm.getWidth() * 1/2; x < bm.getWidth(); x++) {
                    int color = bm.getPixel(x, y);
                    redAvgLeft += ((color & 0xff0000) >> 16) / 100.0; //divide by 100 to prevent overflow
                    blueAvgLeft += (color & 0xff) / 100.0;
                }
            }

            //Total the "amount" of red and blue on the right half of the screen
            double redAvgRight = 0;
            double blueAvgRight = 0;
            for (int y = 0; y < bm.getHeight(); y++) {
                for (int x = 0; x < bm.getWidth() * 1/2; x++) {
                    int color = bm.getPixel(x, y);

                    redAvgRight += ((color & 0xff0000) >> 16) / 100.0;
                    blueAvgRight += (color & 0xff) / 100.0;
                }
            }

            int numPixels = bm.getHeight() * bm.getWidth() / 2;

            redAvgLeft /= numPixels;
            blueAvgLeft /= numPixels;
            redAvgRight /= numPixels;
            blueAvgRight /= numPixels;

            //Positive certaintyForRed corresponds to red probably being on the left half of the screen
            //A higher magnitude indicates a higher probability of being correct
            double certaintyForRed = (redAvgLeft - redAvgRight) - (blueAvgLeft - blueAvgRight);

            //Robot will move for 0.6 seconds in a direction determined in the following if/else block
            double movementTime = 0.6;
            double drivePower = 0;

            //Put the jewel stick down. Get ready to push a jewel off the stand
            robot.jewelStick.setPosition(JEWEL_STICK_MAX); 
            runtime.reset();
            while (runtime.seconds() < 1.1) {
                idle();
            }

            if (certaintyForRed > 0) {
                //red is probably on the left

                //for RED team, the robot should go towards the right of the screen, which is forwards
                //for BLUE team, the robot should go towards the left of the screen, which is backwards
                if (TEAM_COLOR.equals("RED")) {
                    drivePower = 0.2;
                }
                else if (TEAM_COLOR.equals("BLUE")) {
                    drivePower = -0.2;
                }
            }
            else {
                //red is probably on the right

                //for RED team, the robot should go towards the left of the screen, which is backwards
                //for BLUE team, the robot should go towards the right of the screen, which is forwards
                if (TEAM_COLOR.equals("RED")) {
                    drivePower = -0.2;
                }
                else if (TEAM_COLOR.equals("BLUE")) {
                    drivePower = 0.2;
                }
            }

            //Knock off the jewel
            move(drivePower, drivePower, movementTime);

            //Stop for some time to stabilize the robot.
            move(0, 0, 1);

            //Lift the jewel stick to its starting position
            robot.jewelStick.setPosition(JEWEL_STICK_MIN);

            //Go to the pictograph if the robot has to 
            //and sense the pictograph.
            if (drivePower < 0) {
                doVuMark();
            }
            else {
                drivePower *= -1;
                move(drivePower, drivePower, movementTime * 2);
                doVuMark();
            }
        }
    }

    /**
     * Checks if a VuMark (pictograph) is visible and store its value in currentVuMark
     * if one is.
     */
    private void doVuMark() {
        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            //some VuMark is visible
            currentVuMark = vuMark;

            // TODO: Make the robot go parallel to the wall based on pose transformations

            /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
             * it is perhaps unlikely that you will actually need to act on this pose information, but
             * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

            /* We further illustrate how to decompose the pose into useful rotational and
             * translational components */
            if (pose != null) {
                //we can fix the robot's path with this
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;

                telemetry.addData("tX", trans.get(0));
                telemetry.addData("tY", trans.get(1));
                telemetry.addData("tZ", trans.get(2));
                telemetry.addData("rX", rot.firstAngle);
                telemetry.addData("rY", rot.secondAngle);
                telemetry.addData("rZ", rot.thirdAngle);

                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    telemetry.addData("Translation", translation);
                    double radiansToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData("Degrees", radiansToTurn);
                }
            }
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();
    }

    /**
     * Initializes the Vuforia engine
     */
    private void initVuforia() {
        //Uncomment this to show what the camera sees
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        //uncomment this to not show what the camera sees (save battery)
//        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();

        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AVf224j/////AAAAGXlS5fEZWkUuukmNJ278W4N56l4Z/TC6awPG5XTapSLGWsXBBcbc7q+C00X3DfcAs1KmILva7ZKd6OAUyTyZ4fAHK2jrLL56vjoWLOZ1+Gr1ZGya6OYBcQmnbFbUrlGLhnyWtqkIu+RwGApf+LZW18bAaBzo2KOpaZZIaD+UJJ1PqzqtM/v4KH+FXBb4LHN4iHe+q1/gabF8m8Qv+Y2i1407Dre4K/mUp2N+6959a0ZckVqcesMhWtUrljKpie664FXHjYQYPIDQwKiSJfsg12nx4s7rto4ZYmAuTWdcwGZeWHz3gb5rutPgyuG5WiApPnL66MyQNsbA8K1DoK/75pGfY1M2GRzCnzrzenNHLZVt";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = new VuforiaLocalizerImplSubclass(params);
    }

    /**
     * Initializes the VuMark sensing. Requires a relicTrackables.activate() later in the program
     */
    private void initVuMark() {
        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    /**
     * Helper method to decrease code in this class.
     * Fixes the direction of travel, sets the motor powers, waits the
     * designated time, and stops the robot.
     */
    private void move(double leftPower, double rightPower, double time) {
        leftPower *= DIRECTION;
        rightPower *= DIRECTION;

        robot.frontleftMotor.setPower(leftPower);
        robot.backleftMotor.setPower(leftPower);
        robot.frontrightMotor.setPower(rightPower);
        robot.backrightMotor.setPower(rightPower);

        runtime.reset();
        while(runtime.seconds() < time) {
            idle();
        }

        robot.frontleftMotor.setPower(0);
        robot.frontrightMotor.setPower(0);
        robot.backleftMotor.setPower(0);
        robot.backrightMotor.setPower(0);

        runtime.reset();
        while(runtime.seconds() < 0.5) {
            idle();
        }
    }
}
