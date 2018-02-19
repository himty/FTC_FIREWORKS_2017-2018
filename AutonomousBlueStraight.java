package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

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

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTest;

@Autonomous
public class AutonomousBlueStraight extends LinearOpMode {
    HardwareTest robot = new HardwareTest();
    private ElapsedTime runtime = new ElapsedTime();

    //Encodes the robot's starting position to guide the
    //robot throughout the autonomous program
    final String TEAM_COLOR = "BLUE"; //can be RED or BLUE
    final String TEAM_POSITION = "STRAIGHT"; //can be CURVED or STRAIGHT

    //Max is the bottom position for the jewel stick
    //Min is the upper position for the jewel stick
    //To make the stick move farther from the robot, raise this value
    final double JEWEL_STICK_MAX = 0.56;
//    final double JEWEL_STICK_MIN = 0.07;

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
//        robot.rampLeft.setPower(1);
//        robot.rampRight.setPower(1);
//        runtime.reset();
//        while (runtime.seconds() < 3) {
//            idle();
//        }

        //Sense the placement of the jewels,
        //knock off the correct jewel,
        //move to the pictograph, and sense it
        doJewelSensing();
        //moveToCrypto();

        /*
         *Move to the cryptobox.
         *The current if/else structure allows for different robot ending positions
         *when programming the robot to place the block in a certain column
        */
        ///BLUE + STRAIGHT
        if (TEAM_COLOR.equals("BLUE") && TEAM_POSITION.equals("STRAIGHT")) {
            if (currentVuMark == RelicRecoveryVuMark.LEFT) {
                moveForward(0.5, 2.2);
                turnLeft(0.5, 0.9);

                dispenseBlock();


                //throw the block over the robot's head
//                moveForward(0.5, 2.2);
//                turnRight(0.5, 1.1);
//                moveBackward(0.5,0.3);
//
//                robot.rampLeft.setPower(0.5);
//                robot.rampRight.setPower(0.5);
//                runtime.reset();
//                while (runtime.seconds() < 1) {
//                    idle();
//                }
//
//                robot.lifter.setPower(0.1);
//                runtime.reset();
//                while (runtime.seconds() < 2) {
//                    idle();
//                }
//                robot.lifter.setPower(0);
//                robot.rampLeft.setPower(0);
//                robot.rampLeft.setPower(0);

                //for jewel stick
//                moveForward(0.5, 1);
//                moveLeft(0.5, 0.5);
            }
            else if (currentVuMark == RelicRecoveryVuMark.RIGHT) {
                moveForward(0.5, 2.2);
                turnLeft(0.5, 0.9);

                dispenseBlock();
            }
            else {
                moveForward(0.5, 2.2);
                turnLeft(0.5, 0.9);

                dispenseBlock();
            }
        }

        //BLUE + CURVED
        else if (TEAM_COLOR.equals("BLUE") && TEAM_POSITION.equals("CURVED")) {
            if (currentVuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("VuMark", "Left");
                telemetry.update();

                //TODO UNTESTED
                moveForward(0.5, 1.5);
                moveRight(0.5, 0.9);
                turnLeft(0.5, 1.5); //180 degrees

                dispenseBlock();

            }
            else if (currentVuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("VuMark", "RIGHT");
                telemetry.update();
                moveForward(0.5, 1.5);
                moveRight(0.5, 0.9);
                turnLeft(0.5, 1.5); //180 degrees

                dispenseBlock();
            }
            else {
                telemetry.addData("VuMark", "Center");
                telemetry.update();
                moveForward(0.5, 1.5);
                moveRight(0.5, 0.9);
                turnLeft(0.5, 1.5); //180 degrees

                dispenseBlock();
            }
        }

        //RED + STRAIGHT
        else if (TEAM_COLOR.equals("RED") && TEAM_POSITION.equals("STRAIGHT")) {
            if (currentVuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("VuMark", "Left");
                telemetry.update();

                //TODO: UNTESTED
                moveBackward(0.5, 2.2);
                turnLeft(0.5, 0.9);

                dispenseBlock();
//                moveForward(0.3, 2);
//                turnRight(0.3, 0.5);
//                turnLeft(0.5, 0.5);
//                moveForward(0.5, 0.5);
            }
            else if (currentVuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("VuMark", "RIGHT");
                telemetry.update();

                moveBackward(0.5, 2.2);
                turnLeft(0.5, 0.9);

                dispenseBlock();
//                moveForward(0.3, 2);
//                turnRight(0.3, 0.5);
//                turnLeft(0.5, 0.5);
//                moveForward(0.5, 0.5);
            }
            else {
                telemetry.addData("VuMark", "Center");
                telemetry.update();

                moveBackward(0.5, 2.2);
                turnLeft(0.5, 0.9);

                dispenseBlock();
//                moveForward(0.3, 2);
//                turnRight(0.3, 0.5);
//                turnLeft(0.5, 0.5);
//                moveForward(0.5, 0.5);
            }
        }

        //RED + CURVED
        else if (TEAM_COLOR.equals("RED") && TEAM_POSITION.equals("CURVED")) {
            if (currentVuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("VuMark", "Left");
                telemetry.update();
                //robot is facing away from the cryptobox
                moveBackward(0.5, 1.5);
                moveRight(0.5, 0.9);
                turnLeft(0.5, 1.5); //180 degrees

                dispenseBlock();
            }
            else if (currentVuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("VuMark", "RIGHT");
                telemetry.update();
                moveBackward(0.5, 1.5);
                moveRight(0.5, 0.9);
                turnLeft(0.5, 1.5); //180 degrees

                dispenseBlock();
            }
            else {
                telemetry.addData("VuMark", "Center");
                telemetry.update();
                moveBackward(0.5, 1.5);
                moveRight(0.5, 0.9);
                turnLeft(0.5, 1.5); //180 degrees

                dispenseBlock();
            }
        }

        //Lower the block, resetting the robot before
        //tele-op mode.
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
                for (int x = bm.getWidth() * 1 / 2; x < bm.getWidth(); x++) {
                    int color = bm.getPixel(x, y);
                    redAvgLeft += ((color & 0xff0000) >> 16) / 100.0; //divide by 100 to prevent overflow
                    blueAvgLeft += (color & 0xff) / 100.0;
                }
            }

            //Total the "amount" of red and blue on the right half of the screen
            double redAvgRight = 0;
            double blueAvgRight = 0;
            for (int y = 0; y < bm.getHeight(); y++) {
                for (int x = 0; x < bm.getWidth() * 1 / 2; x++) {
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
            double certaintyForBlue = (blueAvgLeft - blueAvgRight) - (redAvgLeft - redAvgRight);

            //Robot will move for 0.6 seconds in a direction determined in the following if/else block
            double movementTime = 0.8;
            double drivePower = 0;

            if (certaintyForRed > 0 || certaintyForRed < 0) {
                //Put the jewel stick down. Get ready to push a jewel off the stand
                lowerJewelStick();

                runtime.reset();
                while (runtime.seconds() < 1.5) {
                    idle();
                }

                if (certaintyForRed > 0) {
                    //red is probably on the left

                    //for RED team, the robot should go towards the right of the screen, which is forwards
                    //for BLUE team, the robot should go towards the left of the screen, which is backwards
                    if (TEAM_COLOR.equals("RED")) {
                        drivePower = 0.2;
                    } else if (TEAM_COLOR.equals("BLUE")) {
                        drivePower = -0.2;
                    }
                } else {
                    //red is probably on the right

                    //for RED team, the robot should go towards the left of the screen, which is backwards
                    //for BLUE team, the robot should go towards the right of the screen, which is forwards
                    if (TEAM_COLOR.equals("RED")) {
                        drivePower = -0.2;
                    } else if (TEAM_COLOR.equals("BLUE")) {
                        drivePower = 0.2;
                    }
                }

                //Knock off the jewel
                if (drivePower > 0) {
                    moveForward(drivePower, movementTime);
                } else
                    moveBackward(-drivePower, movementTime);


                //Stop for some time to stabilize the robot.
                runtime.reset();
                while (runtime.seconds() < 1) {
                    idle();
                }

                //Lift the jewel stick to its starting position
                raiseJewelStick();
            }

            //Go to the pictograph if the robot has to
            //and sense the pictograph.
            if (drivePower < 0) {
                doVuMark();
            } else {
//               move(drivePower, drivePower, movementTime * 2);
                if (drivePower > 0) {
                    //                    moveBackward(drivePower, movementTime * 2);
                    moveBackward(drivePower, movementTime);
                }
                else if (drivePower < 0) {
//                    moveForward(-drivePower, movementTime * 2);
                    moveForward(-drivePower, movementTime);
                }
                else {
                    //drivePower == 0
                    //the robot didn't move or knock off a jewel
                    if (TEAM_COLOR.equals("RED")) {
                        drivePower = 0.2;
                    } else if (TEAM_COLOR.equals("BLUE")) {
                        drivePower = -0.2;
                    }
                    moveForward(drivePower, movementTime);
                }

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
            //doVuMark();
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

                telemetry.addData("VuMark", currentVuMark);

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
    private void moveForward(double power, double time) {
        moveHelper(power, time, Math.atan2(power, 0) - Math.PI / 4);
    }

    private void moveBackward(double power, double time) {
        moveHelper(power, time, Math.atan2(-power, 0) - Math.PI / 4);
    }

    private void moveRight(double power, double time) {
        moveHelper(power, time, Math.atan2(0, -power) - Math.PI / 4);
    }

    private void moveLeft(double power, double time) {
        moveHelper(power, time, Math.atan2(0, power) - Math.PI / 4);
    }

    private void moveHelper(double power, double time, double robotAngle) {
        //Defining which buttons on joystick corresponds to motor left and right power
        //Calculate left and right motor drivetrain power
        /*TELEOP MODE USAGE
//        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        */
        double r = power * DIRECTION;

        final double v1 = r * Math.cos(robotAngle);
        final double v2 = r * Math.sin(robotAngle);
        final double v3 = r * Math.sin(robotAngle);
        final double v4 = r * Math.cos(robotAngle);

        //Set powers
        robot.frontleftMotor.setPower(v1);
        robot.frontrightMotor.setPower(v2);
        robot.backleftMotor.setPower(v3);
        robot.backrightMotor.setPower(v4);

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
    //TURNING, NOT SIDEWAYS; SIDEWAYS IS MOVELEFT
    private void turnLeft(double power, double time) {
        final double v1 = power;
        final double v2 = -power;
        final double v3 = power;
        final double v4 = -power;

        //Set powers
        robot.frontleftMotor.setPower(v1);
        robot.frontrightMotor.setPower(v2);
        robot.backleftMotor.setPower(v3);
        robot.backrightMotor.setPower(v4);

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

    private void turnRight(double power, double time) {
        turnLeft(-power, time);
    }

    private void dispenseBlock() {
        robot.rampLeft.setPower(-0.5);
        robot.rampRight.setPower(-0.5);
        runtime.reset();
        while (runtime.seconds() < 1) {
            idle();
        }

        moveForward(0.5, 0.8);

        robot.rampLeft.setPower(0);
        robot.rampLeft.setPower(0);

        moveBackward(0.5, 0.5);
    }

    private void lowerJewelStick() {
        robot.jewelStick.setPower(-0.2);
        runtime.reset();
        while (runtime.seconds() < 1) {
            idle();
        }
        robot.jewelStick.setPower(0);
    }

    private void raiseJewelStick() {
        robot.jewelStick.setPower(0.2);
        runtime.reset();
        while (runtime.seconds() < 1.1) {
            idle();
        }
        robot.jewelStick.setPower(0);
    }
}
