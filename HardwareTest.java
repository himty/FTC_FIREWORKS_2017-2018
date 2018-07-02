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
    public DcMotor rampLeft = null;
    public DcMotor rampRight = null;
    public DcMotor    jewelStick = null;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTest() {
        ;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save a reference to hwMap
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

        jewelStick = hwMap.dcMotor.get("jewel_stick");
        jewelStick.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jewelStick.setPower(0);
        jewelStick.setDirection(DcMotor.Direction.REVERSE);
        jewelStick.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rampLeft = hwMap.dcMotor.get("ramp_left");
        rampRight = hwMap.dcMotor.get("ramp_right");

        rampLeft.setDirection(DcMotor.Direction.FORWARD);
        rampRight.setDirection(DcMotor.Direction.REVERSE);
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
