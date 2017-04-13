package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.FlickerShooterClass;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.RobotWithWheeledShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.dullesrobotics.ftc.libraries.commonMethods.delay;

/**
 * Created by Kenneth on 11/6/2016.
 */

@TeleOp(name = "ArcadeDriveTeleOp")
public class ArcadeDriveTeleOp extends OpMode {
    //private RobotWithWheeledShooter robotWithWheeledShooter;
    private RobotWithFlickerShooter robotWithFlickerShooter;
    private FlickerShooterClass shooter;
    private ArcadeDrive ArcDrive;
    private String shooterMotor1; //This is for Wheeled Shooter and Flicker Shooter
    private String shooterMotor2; //This is for Wheeled Shooter
    private ServoControllerLib servController;
    private boolean reversed = true;
    private boolean prevStateReverse = reversed;

    //private boolean quickly = true;
    //private boolean prevStateQuickly = quickly;
    private boolean shooting = false;

    @Override
    public void init() {
        /*For Wheeled Shooter

        //robotWithWheeledShooter = new RobotWithWheeledShooter(hardwareMap.dcMotor.get("FLM"),hardwareMap.dcMotor.get("FRM"),hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),gamepad1,hardwareMap.dcMotor.get("leftShooter"),hardwareMap.dcMotor.get("rightShooter"));
        ArcDrive = new ArcadeDrive(robotWithWheeledShooter);
        robotWithWheeledShooter.setDriveTrain(ArcDrive);
        */
        robotWithFlickerShooter = new RobotWithFlickerShooter((hardwareMap.dcMotor.get("FLM"),hardwareMap.dcMotor.get("FRM"),hardwareMap.dcMotor.get("BLM")))
        //robotWithFlickerShooter = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),hardwareMap.dcMotor.get("intake"),gamepad1);
        ArcDrive = new ArcadeDrive(robotWithFlickerShooter);
        robotWithFlickerShooter.setDriveTrain(ArcDrive);
        //servController = new ServoControllerLib(hardwareMap.servo.get("btnServo"));
        //shooter = new FlickerShooterClass(hardwareMap.dcMotor.get("Shooter"),this);
    }

    @Override
    public void loop() {
        boolean curState = robotWithFlickerShooter.getGamepad1().right_bumper;
        DcMotor intake = robotWithFlickerShooter.getBallIntake();
        if (curState && (prevStateReverse == false)&&curState == true){
            reversed = !reversed;
        }
        prevStateReverse = curState;
        if (!reversed) {
            robotWithFlickerShooter.driveWithGamepad();
        } else
        {
            robotWithFlickerShooter.reverseGamepad();
        }
    }
}
