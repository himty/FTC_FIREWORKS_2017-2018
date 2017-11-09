package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareTest
{
//    /* Public OpMode members. */
//    public DcMotor  leftMotor   = null;
//    public DcMotor  rightMotor  = null;
//    public DcMotor  leftMotor2 = null;
//    public DcMotor  rightMotor2 = null;
    public DcMotor  linearSlide = null;
//    public DcMotor  ballHolder  = null;
//    public DcMotor  beaconPusher = null;
////    public DcMotor  popper      = null;
////    public Servo    ballDropper = null;
////    public Servo    beaconPusher= null;
////    public CompassSensor compSensor = null;
////    public LightSensor lightSensor = null;
//
    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTest() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
//        // save reference to HW Map
        hwMap = ahwMap;
//
//        // Define and Initialize Motors
//        leftMotor   = hwMap.dcMotor.get("left_drive");
//        rightMotor  = hwMap.dcMotor.get("right_drive");
//        leftMotor2  = hwMap.dcMotor.get("left_drive2");
//        rightMotor2 = hwMap.dcMotor.get("right_drive2");
//        leftMotor.setDirection(DcMotor.Direction.REVERSE);
//        rightMotor.setDirection(DcMotor.Direction.FORWARD);
//        leftMotor2.setDirection(DcMotor.Direction.REVERSE);
//        rightMotor2.setDirection(DcMotor.Direction.FORWARD);
//
////        popper      = hwMap.dcMotor.get("popper");
        linearSlide = hwMap.dcMotor.get("linear_slide");
////        ballHolder = hwMap.dcMotor.get("ball_holder");
////        beaconPusher = hwMap.dcMotor.get("beacon_pusher");
////        beaconPusher.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // Set all motors to run without encoders.
//        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        ballHolder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        beaconPusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        popper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        ballHolder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        beaconPusher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        popper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        ballHolder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        beaconPusher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        popper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//
//        // Set all motors to zero power
//        leftMotor.setPower(0);
//        leftMotor2.setPower(0);
//        rightMotor.setPower(0);
//        rightMotor2.setPower(0);
//        linearSlide.setPower(0);
////        ballHolder.setPower(0);
////        beaconPusher.setPower(0);
////        popper.setPower(0);
//
//
////        // Define and initialize ALL installed servos.
////        ballDropper = hwMap.servo.get("ball_dropper");
////        beaconPusher = hwMap.servo.get("beacon_pusher");
////
////        ballDropper.setPosition(0);
////        beaconPusher.setPosition(0.5);
//
//
//        //Define sensors
////        compSensor = hwMap.compassSensor.get("compass_sensor");
////        lightSensor = hwMap.lightSensor.get("light_sensor");
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
