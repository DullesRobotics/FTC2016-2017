package com.dullesrobotics.ftc.libraries;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by kk200 on 1/6/2017.
 */

public class ServoControllerLib {
    private Servo buttonServo;

    public ServoControllerLib(Servo btnServ){
        buttonServo = btnServ;
    }

    public ServoControllerLib(Servo btnServ, double degrees){
        buttonServo = btnServ;
        buttonServo.setPosition(degrees);
    }

    public ServoControllerLib(Servo btnServ, Servo.Direction dir){
        buttonServo = btnServ;
        buttonServo.setDirection(dir);
    }

    public void setDegrees(double degrees){
        buttonServo.setPosition(degrees);
    }

    public void setDirection(Servo.Direction dir){
        buttonServo.setDirection(dir);
    }
}
