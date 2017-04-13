package com.dullesrobotics.ftc.libraries;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by kk200 on 4/12/2017.
 *
 * TODO: Implement wheeled shooter motors, I wasnt sure on the set up so I skipped that
 *
 */

public class AdvancedRobot extends BasicRobot {
    private DcMotor strifeMotor;
    private DcMotor ballIntake;

    /**
    *ORDER: FrontRight, FrontLeft, BackRight, BackLeft, Strife, Intake
    */

    public AdvancedRobot(DcMotor[] motors, Gamepad g1){
        super(motors, g1);
        if (motors[4] != null) this.strifeMotor = motors[4];
        if (motors[5] != null) this.ballIntake = motors[5];
    }

    public DcMotor getFrontRight(){ return super.getFRM(); }
    public DcMotor getFrontLeft(){ return super.getFLM(); }
    public DcMotor getBackRight(){ return super.getBRM(); }
    public DcMotor getBackLeft(){ return super.getBLM(); }
    public DcMotor getStrifeMotor(){ return strifeMotor; }
    public DcMotor getBallIntake(){ return ballIntake; }

    public DcMotor getFRM(){ return super.getFRM(); }
    public DcMotor getFLM(){ return super.getFLM(); }
    public DcMotor getBRM(){ return super.getBRM(); }
    public DcMotor getBLM(){ return super.getBLM(); }

    public void setGamepad(Gamepad g1){ super.setGamepad1(g1); }

    public Gamepad getGamepad(){ return super.getGamepad1(); }

    public void setDriveTrain(TeleOpDrivetrain driveTrain){ super.setDriveTrain(driveTrain); }

    public void drive(){ super.getDriveTrain().driveWithGamepad(); }
    public void reverseDrive(){ super.getDriveTrain().reverseGamepad(); }
}
