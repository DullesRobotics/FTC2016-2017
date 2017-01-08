package com.dullesrobotics.ftc.libraries;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * Created by Kenneth on 1/8/2017.
 */

public class SensorListener implements SensorEventListener {
    private SensorManager sensorManager;
    private Sensor accelerometer;
    private Sensor magnetometer;

    private float[] acceleration;
    private float[] magOutput;
    private float[] orientation;
    private float[] startOrientation = null;

    //private float pitch;
    //private float roll;
    private float yaw;

    public final int LANDSCAPE_RIGHT = 0;//Orientation you get when you rotate portrait phone right 90deg to landscape
    public final int LANDSCAPE_LEFT = 1;//Orientation you get when you rotate portrait phone left 90deg to landscape
    public final int PORTRAIT_VERTICAL = 2; //Normal Portrait Orientation
    public final int UPSIDE_DOWN_PORTRAIT = 3;//Upside down portrait

    private int phoneOrientation; //defaults to PORTRAIT orientation

    public SensorListener(OpMode opMode, int phoneOrient){
        sensorManager = (SensorManager)(opMode.hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE));
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        phoneOrientation = phoneOrient;
    }

    public void resetStartOrientation(){
        startOrientation = null;
    }

    public void register(){
        sensorManager.registerListener(this,accelerometer, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_GAME);
    }

    public void unregister(){
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
            acceleration = event.values;
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD){
            magOutput = event.values;
        }

        if(acceleration != null && magOutput != null){
            float[] R = new float[9];
            float[] I = new float[9];
            boolean success = SensorManager.getRotationMatrix(R, I, acceleration,magOutput);
            if(success){
                SensorManager.getOrientation(R,orientation);
                if(startOrientation == null){
                    startOrientation = new float[orientation.length];
                    System.arraycopy(orientation,0, startOrientation, 0, orientation.length);
                }
                updateYaw();
            }
        }
    }

    private void updateYaw(){
        if(orientation != null && startOrientation != null){
            switch (phoneOrientation){//Make clockwise positive
                case UPSIDE_DOWN_PORTRAIT:
                    yaw = (orientation[2] - startOrientation[2]);
                    break;
                case LANDSCAPE_LEFT:
                    yaw = (orientation[1] - startOrientation[1])*-1;//neg
                    break;
                case  LANDSCAPE_RIGHT:
                    yaw = orientation[1] - startOrientation[1];
                    break;
                case PORTRAIT_VERTICAL:
                    yaw = (orientation[2] - startOrientation[2])*-1;
            }

        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public float[] getOrientation() {
        return orientation;
    }

    public float[] getStartOrientation() {
        return startOrientation;
    }

    public int getPhoneOrientation() {
        return phoneOrientation;
    }

    public void setPhoneOrientation(int phoneOrientation) {
        this.phoneOrientation = phoneOrientation;
    }

    public float getYaw() {
        return yaw;
    }
}
