package com.dullesrobotics.ftc.libraries;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Kenneth on 12/27/2016.
 */

public class RobotWithFlickerShooter extends BasicRobot {
    private DcMotor shooterMotor;
    private DcMotor lift;
    public RobotWithFlickerShooter(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gamepad g1, DcMotor shooter){
        super(frontLeft,frontRight,backLeft,backRight, g1);
        shooterMotor = shooter;
    }
    public RobotWithFlickerShooter(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gamepad g1, Gamepad g2,DcMotor shooter){
        super(frontLeft,frontRight,backLeft,backRight, g1);
        shooterMotor = shooter;
    }

    public RobotWithFlickerShooter(DcMotor backLeft, DcMotor backRight, Gamepad g1, DcMotor shooter){
        super(backLeft,backRight, g1);
        shooterMotor = shooter;
    }

    /*
    public RobotWithFlickerShooter (DcMotor backLeft, DcMotor backRight, Gamepad g1, DcMotor lift){
        super(backLeft,backRight, g1);
        lift =
    }
*/
    public RobotWithFlickerShooter(DcMotor shooter,Gamepad g1){
        super(g1);
        shooterMotor = shooter;
    }

    public RobotWithFlickerShooter(Gamepad g1){
        super(g1);
    }

    public DcMotor getShooterMotor() {
        return shooterMotor;
    }

    public void setShooterMotor(DcMotor shooterMotor) {
        this.shooterMotor = shooterMotor;
    }

    public void turnForwards(float power){
        shooterMotor.setPower(power);
    }
    public void turnBackwards(float power){
        shooterMotor.setPower(power);
    }
    public void turnForwards(){
        shooterMotor.setPower(1.0);
    }
    public void turnBackwards(){
        shooterMotor.setPower(-1.0);
    }
    public void stopShooter(){
        shooterMotor.setPower(0.0);
    }



}
