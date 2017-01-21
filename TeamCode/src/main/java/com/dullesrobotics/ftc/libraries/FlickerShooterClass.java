package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by nimir on 1/20/2017.
 */

public class FlickerShooterClass {
    DcMotor flickerMotor;
    boolean reversed = false;
    private ElapsedTime runtime;
    private OpMode opMode;

    public FlickerShooterClass(DcMotor flickerMtr, OpMode op) {
        flickerMotor = flickerMtr;
        opMode = op;
        runtime  = new ElapsedTime();
        flickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setSpeed(double speed){
        if (reversed) {
            flickerMotor.setPower(speed);
        } else {
            flickerMotor.setPower(-speed);
        }
    }

    public void runMotorFullSpeed() {
        flickerMotor.setPower(1);
        /*runtime.reset();
        while (runtime.seconds() < time){
            opMode.waitOneFullHardwareCycle();
        }
        flickerMotor.setPower(0);*/
    }

    public void stopMotor(){
        flickerMotor.setPower(0);
    }

    public void releaseMotor(){
        flickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void reverseMotor(){
        reversed = !reversed;
    }

    public void setMotorMode(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        flickerMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
