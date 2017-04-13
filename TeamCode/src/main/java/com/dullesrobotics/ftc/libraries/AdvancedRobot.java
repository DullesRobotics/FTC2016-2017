package com.dullesrobotics.ftc.libraries;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by kk200 on 4/12/2017.
 *
 * TODO: Implement wheeled shooter motors, I wasnt sure on the set up so I skipped that
 *
 * PS: This doesnt implement two controllers since we never use two controllers, but its easy to add.
 */

public class AdvancedRobot {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor strifeMotor;
    private DcMotor ballIntake;
    private Gamepad g1;
    private TeleOpDrivetrain driveTrain;

    //No Strife Motor, No Intake
    public AdvancedRobot(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, Gamepad g1){
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.g1 = g1;
    }

    //No Intake
    public AdvancedRobot(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft,DcMotor strifeMotor, Gamepad g1){
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.strifeMotor = strifeMotor;
        this.g1 = g1;
    }

    //Has everything
    public AdvancedRobot(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft,DcMotor strifeMotor,DcMotor ballIntake, Gamepad g1){
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.strifeMotor = strifeMotor;
        this.ballIntake = ballIntake;
        this.g1 = g1;
    }

    /**
    *ORDER: FrontRight, FrontLeft, BackRight, BackLeft, Strife, Intake
    */

    public AdvancedRobot(DcMotor[] motors, Gamepad g1){
        if (motors[0] != null) this.frontRight = motors[0];
        if (motors[1] != null) this.frontLeft = motors[1];
        if (motors[2] != null) this.backRight = motors[2];
        if (motors[3] != null) this.backLeft = motors[3];
        if (motors[4] != null) this.strifeMotor = motors[4];
        if (motors[5] != null) this.ballIntake = motors[5];
        this.g1 = g1;
    }

    public DcMotor getFrontRight(){ return frontRight; }
    public DcMotor getFrontLeft(){ return frontLeft; }
    public DcMotor getBackRight(){ return backRight; }
    public DcMotor getBackLeft(){ return backLeft; }
    public DcMotor getStrifeMotor(){ return strifeMotor; }
    public DcMotor getBallIntake(){ return ballIntake; }

    public void setGamepad(Gamepad g1){ this.g1 = g1; }
    public Gamepad getGamepad(){ return g1; }

    public void setDriveTrain(TeleOpDrivetrain driveTrain){ this.driveTrain = driveTrain; }

    public void drive(){ driveTrain.driveWithGamepad(); }
    public void reverseDrive(){ driveTrain.reverseGamepad(); }
}
