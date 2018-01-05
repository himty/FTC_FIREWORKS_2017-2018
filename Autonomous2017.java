package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import static com.sun.tools.javac.util.Constants.format;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTest;

/**
 * Created by Queen on 10/30/16.
 */
@Autonomous
public class Autonomous2017 extends LinearOpMode {
    HardwareTest robot = new HardwareTest();
    private ElapsedTime runtime = new ElapsedTime();

    double offsetTime = 0;

    /*
     * Vuforia variables
     */
    RelicRecoveryVuMark currentVuMark = RelicRecoveryVuMark.UNKNOWN;
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizerImplSubclass vuforia; //stores our instance of the Vuforia localization engine
    File directory;
    //    VuforiaTrackables beacons;
    int fileCount = 1;

    final String TEAM_COLOR = "blue";

    static final double FORWARD_SPEED = -1;
    static final double TURN_SPEED = 1;
    static final double POWER = 1;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        initVuforia();
        robot.accelSensor = new Accelerometer(hardwareMap);

        initVuMark();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (runtime.seconds() < 6.5) {
            //wait for the camera to boot
            idle();
        }

        relicTrackables.activate();

        telemetry.addData("Status", "Running");
        telemetry.update();

        //do jewel sensing

        //go forwards or backwards
        offsetTime = 0.5;

        while (true) {
            doVuMark();
        }

        //go to the VuMark
//        runtime.reset();
//        //make this condition based off of when the pose's x is near 0 or something
//        while(currentVuMark == RelicRecoveryVuMark.UNKNOWN
//                && runtime.seconds() < 100000 + offsetTime) {
//            doVuMark();
//        }
//
//        //go to the box thing
//        runtime.reset();
//        while(currentVuMark != RelicRecoveryVuMark.UNKNOWN
//                && runtime.seconds() < 1) {
//            doVuMark();
//        }
    }

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

            // make the robot go parallel to the wall

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
     * @return whether initialization was successful
     */
    private void initVuforia() {
        //uncomment this to show what the camera sees
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        //uncomment this to not show what the camera sees (save battery)
//        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();

        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AVf224j/////AAAAGXlS5fEZWkUuukmNJ278W4N56l4Z/TC6awPG5XTapSLGWsXBBcbc7q+C00X3DfcAs1KmILva7ZKd6OAUyTyZ4fAHK2jrLL56vjoWLOZ1+Gr1ZGya6OYBcQmnbFbUrlGLhnyWtqkIu+RwGApf+LZW18bAaBzo2KOpaZZIaD+UJJ1PqzqtM/v4KH+FXBb4LHN4iHe+q1/gabF8m8Qv+Y2i1407Dre4K/mUp2N+6959a0ZckVqcesMhWtUrljKpie664FXHjYQYPIDQwKiSJfsg12nx4s7rto4ZYmAuTWdcwGZeWHz3gb5rutPgyuG5WiApPnL66MyQNsbA8K1DoK/75pGfY1M2GRzCnzrzenNHLZVt";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = new VuforiaLocalizerImplSubclass(params);
    }

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
}