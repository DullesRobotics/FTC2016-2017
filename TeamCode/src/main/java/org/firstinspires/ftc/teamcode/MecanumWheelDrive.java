package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.FlickerShooterClass;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.RobotWithMecanumWheels;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by kk200 on 2/7/2017.
 */
@TeleOp(name = "MecamumWheelDrive")
public class MecanumWheelDrive extends OpMode{
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
    //private boolean twoDrivers = true;

    @Override
    public void init() {
        /*For Wheeled Shooter
        //robotWithWheeledShooter = new RobotWithWheeledShooter(hardwareMap.dcMotor.get("FLM"),hardwareMap.dcMotor.get("FRM"),hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),gamepad1,hardwareMap.dcMotor.get("leftShooter"),hardwareMap.dcMotor.get("rightShooter"));
        ArcDrive = new ArcadeDrive(robotWithWheeledShooter);
        robotWithWheeledShooter.setDriveTrain(ArcDrive);
        */

        robotWithFlickerShooter = new RobotWithMecanumWheels(hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),gamepad1);
        ArcDrive = new ArcadeDrive(robotWithFlickerShooter);
        robotWithFlickerShooter.setDriveTrain(ArcDrive);
        servController = new ServoControllerLib(hardwareMap.servo.get("btnServo"));
        //shooter = new FlickerShooterClass(hardwareMap.dcMotor.get("Shooter"),this);
    }

    @Override
    public void loop() {

        /*          UNCOMMENT FOR WHEELED SHOOTER
        robotWithWheeledShooter.driveWithGamepad();
        if (robotWithWheeledShooter.getGamepad1().a)  //change
            robotWithWheeledShooter.shootForward();
        else if(robotWithWheeledShooter.getGamepad1().b) //change
            robotWithWheeledShooter.shootBackward();
        else if(robotWithWheeledShooter.getGamepad1().x) //change
            robotWithWheeledShooter.stopShooter();
            */
        boolean curState = robotWithFlickerShooter.getGamepad1().right_bumper;
        //boolean curStateQuickly = robotWithFlickerShooter.getGamepad1().left_bumper;
        /*if (curStateQuickly && (prevStateQuickly == false)&&curStateQuickly == true){
            quickly = !quickly;
        }*/
        if (curState && (prevStateReverse == false)&&curState == true){
            reversed = !reversed;
        }
        prevStateReverse = curState;
        //prevStateQuickly = curStateQuickly;
        if (!reversed) {
            robotWithFlickerShooter.driveWithGamepad();
        } else
        {
            robotWithFlickerShooter.reverseGamepad();
        }

        /*if (robotWithFlickerShooter.getGamepad1().left_bumper && !shooting){
            shooting = true;
            shooter.runMotorFullSpeed();
            delay(1000);
            shooter.stopMotor();
            delay(250);
            shooter.releaseMotor();
            shooting = false;
        }*/

        /*if (robotWithFlickerShooter.getGamepad1().right_trigger > 0)  //change
        {
            robotWithFlickerShooter.turnForwards(robotWithFlickerShooter.getGamepad1().right_trigger);
        }
        else if(robotWithFlickerShooter.getGamepad1().left_trigger > 0) //change
        {
            robotWithFlickerShooter.turnBackwards(-robotWithFlickerShooter.getGamepad1().left_trigger);
        }
        else if (robotWithFlickerShooter.getGamepad1().dpad_up) {
            //Move Lift Up
        }
        else if (robotWithFlickerShooter.getGamepad1().dpad_down){
            //Move Lift Down
        }
        else {
            robotWithFlickerShooter.stopShooter();
        }*/

        if (robotWithFlickerShooter.getGamepad1().dpad_right){
            servController.setDegrees(180); //Right
        } else if (robotWithFlickerShooter.getGamepad1().dpad_left){
            servController.setDegrees(0); //Left
        }
    }

}