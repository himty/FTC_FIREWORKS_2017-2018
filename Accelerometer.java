package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by jenny on 12/21/2017.
 */

public class Accelerometer{
    private Sensor accelSensor;

    private double xZero;
    private double yZero;
    private double zZero;
    private double scale;

    private double xAccel;
    private double yAccel;
    private double zAccel;

    public Accelerometer(HardwareMap hardwareMap) {
//        xZero = -0.007787; // raw xAccel when phone is flat on floor
//        yZero = -0.021964; // raw yAccel when phone is flat on the floor
//        zZero = -0.979762; // raw zAccel when phone is vertical
        xZero = 0.0814840;
        yZero = -0.214747;
        zZero = 0.80818510;
        scale = 1;
//        scale =  9.62800997; // raw zAccel when phone is flat on the floor
        initSensor(hardwareMap);
    }


    /**
     * Initializes the Accelerometer
     * Creates the event listener and registers the sensor
     * @param hardwareMap the hardware map of the robot
     */
    public void initSensor(HardwareMap hardwareMap) {
        SensorEventListener accelListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                xAccel = (event.values[0] - xZero);
                yAccel = (event.values[1] - yZero);
                zAccel = (event.values[2]);

                double pitch = Math.atan(xAccel/Math.sqrt(Math.pow(yAccel,2)+Math.pow(zAccel,2)));
                //keep copying
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {
                // not using this method
            }
        };

        SensorManager sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        sensorManager.registerListener(accelListener, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_GAME);
        accelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    }

    /**
     * Returns the scaled x acceleration
     * @return xAccel scaled
     */
    public double getxAccel() {
        return xAccel;
    }

    /**
     * Returns the scaled y acceleration
     * @return yAccel scaled
     */
    public double getyAccel() {
        return yAccel;
    }

    /**
     * Returns the scaled z acceleration
     * @return zAccel scaled
     */
    public double getzAccel() {
        return zAccel;
    }

    public String toString() {
        return xAccel + " " + yAccel + " " + zAccel;
    }
}
