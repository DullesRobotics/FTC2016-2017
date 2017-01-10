package com.dullesrobotics.ftc.libraries;

import com.dullesrobotics.ftc.mods.SensorListener;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Kenneth on 1/8/2017.
 */

public class AutonomousDrive {
    BasicRobot robot;
    final double ENCODERTICKSPERREVOLUTION = 1478.4;
    final double CIRCUMFERENCEOFWHEELCENTIMETERS = Math.PI*9.6;
    final double TICKSPERCENTIMETER = CIRCUMFERENCEOFWHEELCENTIMETERS/ENCODERTICKSPERREVOLUTION;
    SensorListener sensorListener;
    float heading;

    public AutonomousDrive(BasicRobot r, SensorListener s) {
        robot = r;
        sensorListener = s;
    }

    public void drive(double power, boolean useOrientationSensor, boolean useEncoders){

    }

    public void driveMantainHeading(double power, int distanceInCM){
        heading = sensorListener.getYaw();
        resetEncoders();
        setRUNWITHENCODERS();

        int Targetticks = (int) (distanceInCM * TICKSPERCENTIMETER);




    }




    public void setRUNWITHENCODERS(){
        robot.getBLM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setRUNWITHOUTENCODERS(){
        robot.getBLM().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setRUNTOPOSITION(){
        robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetEncoders(){
        robot.getBLM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
