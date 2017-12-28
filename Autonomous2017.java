package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTest;

/**
 * Created by Queen on 10/30/16.
 */
@Autonomous
public class Autonomous2017 extends LinearOpMode {
    HardwareTest robot = new HardwareTest();
    private ElapsedTime runtime = new ElapsedTime();

/*
    double RED_MIN = 0.255, RED_MAX = 0.285;        //mean = 0.28
    double BLUE_MIN = 0.21, BLUE_MAX = 0.25;      //mean = 0.23
*/

    final String TEAM = "Team TBD (gasp)";

    static final double FORWARD_SPEED = -1;
    static final double TURN_SPEED = 1;
    static final double POWER = 1;

    //int moveNum = 0;

    //final int SIGN = -1;

    int i;
    //double temp;

    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }


    // first step: claw catches the block
    void throwball() {
        robot.clawLeft.setPosition(0.8);
        runtime.reset();
        while (runtime.seconds() < 1) {
            idle();
        }

/*        // Start
        //Move forward
        setPowers(FORWARD_SPEED, FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        //Turn left towards first beacon
        setPowers(-FORWARD_SPEED, FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        //Go towards first beacon
        setPowers(FORWARD_SPEED, FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        //Turn right to position in front of first beacon
        setPowers(FORWARD_SPEED, -FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.13)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        //Do stuff to beacon
        //doBeaconAction();

        //Move to next beacon -- NOT STRAIGHT
        setPowers(FORWARD_SPEED, FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }


        // Step 4:  Stop and close the claw.
        setPowers(0, 0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
        idle();
    }*/
/*

    void setPowers(double left, double right) {
        robot.leftMotor.setPower(left);
        robot.leftMotor2.setPower(left);
        robot.rightMotor.setPower(right);
        robot.rightMotor2.setPower(right);
    }
*/


    }
}

    /*void doBeaconAction() {
        temp = 0;
        for (i = 0; i < 100; i++) {
            temp += robot.lightSensor.getLightDetected();
        }
        temp /= 100;

        if (temp <= RED_MAX && temp >= RED_MIN) { //is red
            if (TEAM.equals("red")) {
                telemetry.addData("Beacon", "Red: push this %.2f", temp);
                telemetry.update();
//                pushButton();
                //moveToOtherButton();
            } else {
                telemetry.addData("Beacon", "Red: push other %.2f", temp);
                telemetry.update();
//                moveToOtherButton();
//                pushButton();
            }
        } else if (temp <= BLUE_MAX && temp >= BLUE_MIN) { //is blue
            if (TEAM.equals("blue")) {
                telemetry.addData("Beacon", "Blue: push this %.2f", temp);
                telemetry.update();
//                pushButton();
                //moveToOtherButton();
            } else {
                telemetry.addData("Beacon", "Blue: push other %.2f", temp);
                telemetry.update();
//                moveToOtherButton();
//                pushButton();
            }
        } else {
            telemetry.addData("Error", "Indistinguishable beacon color %.2f", temp);
            telemetry.update();
            moveToOtherButton();
        }
    }

    ///

    {

        //Go forwards
        setPowers(FORWARD_SPEED, FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();

        }
    }

    void grabjewel() {
        robot.claw.setPower(0.8);
        runtime.reset();
        while (runtime.seconds() < 1.5) { //TODO test this
            idle();
        }



        //Go backwards
        setPowers(-FORWARD_SPEED, FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();

            //Turn right towards jewel column
            setPowers(TURN_SPEED, TURN_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.8)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
                idle();


//add go forward


//*//*                ///
//
//                robot.beaconPusher.setPower(-0.2);
//                runtime.reset();
//                while (runtime.seconds() < 2) {
//                    idle();
//                }
//
//                robot.beaconPusher.setPower(0);
//            }
//
//        void moveToOtherButton() {
//            setPowers(FORWARD_SPEED, FORWARD_SPEED);
//            runtime.reset();
//            while (runtime.seconds() < 0.3) {
//                ;
//            }
//            setPowers(0,0);
//        }
//    }*/
