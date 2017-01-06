package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Kenneth on 12/27/2016.
 */
public class RobotWithWheeledShooter extends BasicRobot {

    private DcMotor shooterMotor;
    public RobotWithWheeledShooter(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gamepad g1, DcMotor shooter){
        super(frontLeft,frontRight,backLeft,backRight, g1);
        shooterMotor = shooter;
    }
    public RobotWithWheeledShooter(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gamepad g1, Gamepad g2,DcMotor shooter){
        super(frontLeft,frontRight,backLeft,backRight, g1);
        shooterMotor = shooter;
    }
    public RobotWithWheeledShooter(DcMotor backLeft, DcMotor backRight, Gamepad g1, DcMotor shooter){
        super(backLeft,backRight, g1);
        shooterMotor = shooter;
    }

    public RobotWithWheeledShooter(DcMotor shooter,Gamepad g1){
        super(g1);
        shooterMotor = shooter;
    }

    public DcMotor getShooterMotor() {
        return shooterMotor;
    }

    public void setShooterMotor(DcMotor shooterMotor) {
        this.shooterMotor = shooterMotor;
    }

    public void turnForwards(){
        shooterMotor.setPower(1.0);
    }
    public void turnBackwards(){
        shooterMotor.setPower(-1.0);
    }
    public void turnForwards(double power){
        shooterMotor.setPower(power);
    }
    public void turnBackwards(double power){
        shooterMotor.setPower(-1*power);
    }

    public void stopShooter(){
        shooterMotor.setPower(0.0);
    }
    /*
    private DcMotor leftShooterMotor;
    private DcMotor rightShooterMotor;
    private double LeftMotorSpeed, RightMotorSpeed = 0;
    public RobotWithWheeledShooter(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gamepad g1, DcMotor leftShooter, DcMotor rightShooter) {
        super(frontLeft, frontRight, backLeft, backRight, g1);
        leftShooterMotor = leftShooter;
        rightShooterMotor = rightShooter;
    }

    public RobotWithWheeledShooter(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gamepad g1, Gamepad g2,DcMotor leftShooter, DcMotor rightShooter) {
        super(frontLeft, frontRight, backLeft, backRight, g1, g2);
        leftShooterMotor = leftShooter;
        rightShooterMotor = rightShooter;
    }


    public DcMotor getLeftShooterMotor() {
        return leftShooterMotor;
    }

    public void setLeftShooterMotor(DcMotor leftShooterMotor) {
        this.leftShooterMotor = leftShooterMotor;
    }

    public DcMotor getRightShooterMotor() {
        return rightShooterMotor;
    }

    public void setRightShooterMotor(DcMotor rightShooterMotor) {
        this.rightShooterMotor = rightShooterMotor;
    }


    /*** Wheeled shooter methods below ***/
    /*
    public void setShooterSpeed(double speed){
        LeftMotorSpeed = speed1;
        RightMotorSpeed = speed2;
    }

    public void shootForward(){ //idk if this will shoot forward or backward, depends on motor
        leftShooterMotor.setPower(LeftMotorSpeed);
        rightShooterMotor.setPower(-RightMotorSpeed);
    }

    public void shootBackward(){ //why would you shoot backward you ask? idk. maybe if the robot is turned.
        leftShooterMotor.setPower(-LeftMotorSpeed);
        rightShooterMotor.setPower(RightMotorSpeed);
    }


    public void stopShooter(){
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);
    }
    */
}
