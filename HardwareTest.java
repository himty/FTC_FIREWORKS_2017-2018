package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareTest
{
    /* Public OpMode members. */
    public DcMotor  frontleftMotor   = null;
    public DcMotor  frontrightMotor  = null;
    public DcMotor  backleftMotor = null;
    public DcMotor  backrightMotor = null;
    public DcMotor  lifter = null;
    //public DcMotor  linearSlide = null;
    public Servo    clawLeft = null;
    public Servo    clawRight = null;

    public DcMotor rampLeft = null;
    public DcMotor rampRight = null;

    public DcMotor    jewelStick = null;

    public DcMotor  RollArm   = null;
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

        // Define and Initialize Motors
        frontleftMotor   = hwMap.dcMotor.get("frontleft_drive");
        frontrightMotor  = hwMap.dcMotor.get("frontright_drive");
        backleftMotor  = hwMap.dcMotor.get("backleft_drive");
        backrightMotor = hwMap.dcMotor.get("backright_drive");
        frontleftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontrightMotor.setDirection(DcMotor.Direction.REVERSE);
        backleftMotor.setDirection(DcMotor.Direction.FORWARD);
        backrightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontleftMotor.setPower(0);
        frontrightMotor.setPower(0);
        backleftMotor.setPower(0);
        backrightMotor.setPower(0);

        lifter = hwMap.dcMotor.get("lifter");
        lifter.setDirection(DcMotor.Direction.FORWARD);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        clawLeft = hwMap.servo.get("claw_left");
//        clawRight = hwMap.servo.get("claw_right");
        //linearSlide = hwMap.dcMotor.get("linear_slide");
        jewelStick = hwMap.dcMotor.get("jewel_stick");
        jewelStick.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jewelStick.setPower(0);
        jewelStick.setDirection(DcMotor.Direction.REVERSE);
        jewelStick.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //RollArm = hwMap.dcMotor.get("roll_arm");

        rampLeft = hwMap.dcMotor.get("ramp_left");
        rampRight = hwMap.dcMotor.get("ramp_right");

        rampLeft.setDirection(DcMotor.Direction.FORWARD);
        rampRight.setDirection(DcMotor.Direction.REVERSE);

        //RollArm.setDirection(DcMotor.Direction.FORWARD);
        //linearSlide.setDirection(DcMotor.Direction.REVERSE);

//        clawLeft.setPosition(0.5);
//        clawRight.setPosition(0.5);

        //jewelStick.setPower(0.5);
        //RollArm.setPower(0);
//
////        popper      = hwMap.dcMotor.get("popper");

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
        //linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.CompassSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.LightSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class HardwareTest
//{
//    /* Public OpMode members. */
//    public DcMotor  frontleftMotor   = null;
//    public DcMotor  frontrightMotor  = null;
//    public DcMotor  backleftMotor = null;
//    public DcMotor  backrightMotor = null;
//    //    public DcMotor  linearSlide = null;
//    public Servo    clawLeft = null;
//    public Servo    clawRight = null;
//

//
//    public Servo    jewelStick = null;
//
//
//    //    public DcMotor  rollArm   = null;
//    //    public DcMotor  ballHolder  = null;
////    public DcMotor  beaconPusher = null;
//////    public DcMotor  popper      = null;
//////    public Servo    ballDropper = null;
//////    public Servo    beaconPusher= null;
//////    public CompassSensor compSensor = null;
//////    public LightSensor lightSensor = null;
////
//    /* Local OpMode members. */
//    HardwareMap hwMap  = null;
//    private ElapsedTime period  = new ElapsedTime();
//
//    /* Constructor */
//    public HardwareTest() {
//    }
//
//    /* Initialize standard Hardware interfaces */
//    public void init(HardwareMap ahwMap) {
////        // save reference to HW Map
//        hwMap = ahwMap;
//
//        // Define and Initialize Motors
//        frontleftMotor   = hwMap.dcMotor.get("frontleft_drive");
//        frontrightMotor  = hwMap.dcMotor.get("frontright_drive");
//        backleftMotor  = hwMap.dcMotor.get("backleft_drive");
//        backrightMotor = hwMap.dcMotor.get("backright_drive");
//        frontleftMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontrightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backleftMotor.setDirection(DcMotor.Direction.FORWARD);
//        backrightMotor.setDirection(DcMotor.Direction.REVERSE);
//
////        lifterLeft = hwMap.servo.get("lifter_left");
////        lifterRight = hwMap.servo.get("lifter_right");
//
//
//        clawLeft = hwMap.servo.get("claw_left");
//        clawRight = hwMap.servo.get("claw_right");
////        linearSlide = hwMap.dcMotor.get("linear_slide");
//        jewelStick = hwMap.servo.get("jewel_stick");
////        rollArm = hwMap.dcMotor.get("roll_arm");
//

//
////        rollArm.setDirection(DcMotor.Direction.FORWARD);
////        linearSlide.setDirection(DcMotor.Direction.REVERSE);
//
//        clawLeft.setPosition(0.5);
//        clawRight.setPosition(0.5);
//        jewelStick.setPosition(0.01);
////        blockCollectorLeft.setPower(0);
////        blockCollectorRight.setPower(0);
//
////        lifterLeft.setPosition(0.5);
////        lifterRight.setPosition(0.5);
//
//        //RollArm.setPower(0);
////
//////        popper      = hwMap.dcMotor.get("popper");
//
//////        ballHolder = hwMap.dcMotor.get("ball_holder");
//////        beaconPusher = hwMap.dcMotor.get("beacon_pusher");
//////        beaconPusher.setDirection(DcMotorSimple.Direction.REVERSE);

////
////        // Set all motors to zero power
////        leftMotor.setPower(0);
////        leftMotor2.setPower(0);
////        rightMotor.setPower(0);
////        rightMotor2.setPower(0);
////        linearSlide.setPower(0);
//////        ballHolder.setPower(0);
//////        beaconPusher.setPower(0);
//////        popper.setPower(0);
////
////
//////        // Define and initialize ALL installed servos.
//////        ballDropper = hwMap.servo.get("ball_dropper");
//////        beaconPusher = hwMap.servo.get("beacon_pusher");
//////
//////        ballDropper.setPosition(0);
//////        beaconPusher.setPosition(0.5);
////
////
////        //Define sensors
//////        compSensor = hwMap.compassSensor.get("compass_sensor");
//////        lightSensor = hwMap.lightSensor.get("light_sensor");
//    }
