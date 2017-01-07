package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.RobotWithWheeledShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Kenneth on 11/6/2016.
 */
@TeleOp(name = "ArcadeDriveTeleOp")
public class ArcadeDriveTeleOp extends OpMode {
    //private RobotWithWheeledShooter robotWithWheeledShooter;
    private RobotWithFlickerShooter robotWithFlickerShooter;
    private ArcadeDrive ArcDrive;
    private String shooterMotor1; //This is for Wheeled Shooter and Flicker Shooter
    private String shooterMotor2; //This is for Wheeled Shooter
    private ServoControllerLib servController;
    private boolean reversed = false;
    private boolean prevStateReverse = false;

    @Override
    public void init() {
        /*For Wheeled Shooter
        //robotWithWheeledShooter = new RobotWithWheeledShooter(hardwareMap.dcMotor.get("FLM"),hardwareMap.dcMotor.get("FRM"),hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),gamepad1,hardwareMap.dcMotor.get("leftShooter"),hardwareMap.dcMotor.get("rightShooter"));
        ArcDrive = new ArcadeDrive(robotWithWheeledShooter);
        robotWithWheeledShooter.setDriveTrain(ArcDrive);
        */

        robotWithFlickerShooter = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),gamepad1,hardwareMap.dcMotor.get("flickerShooter"));
        //robotWithFlickerShooter = new RobotWithFlickerShooter(gamepad1);
        ArcDrive = new ArcadeDrive(robotWithFlickerShooter);
        robotWithFlickerShooter.setDriveTrain(ArcDrive);
        servController = new ServoControllerLib(hardwareMap.servo.get("BtnServo"));
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
        if (curState && (prevStateReverse == false)&&prevStateReverse!=curState){
            reversed = !reversed;
        }
        prevStateReverse = curState;
        if (!reversed) {
            robotWithFlickerShooter.driveWithGamepad();
        } else
        {
            robotWithFlickerShooter.reverseGamepad();
        }

        if (robotWithFlickerShooter.getGamepad1().right_trigger > 0)  //change
        {
            robotWithFlickerShooter.turnForwards(robotWithFlickerShooter.getGamepad1().right_trigger);
        }
        else if(robotWithFlickerShooter.getGamepad1().left_trigger > 0) //change
        {
            robotWithFlickerShooter.turnBackwards(robotWithFlickerShooter.getGamepad1().left_trigger);
        }
        else if (robotWithFlickerShooter.getGamepad1().dpad_up) {
            //Move Lift Up
        }
        else if (robotWithFlickerShooter.getGamepad1().dpad_down){
            //Move Lift Down
        }
        else {
            robotWithFlickerShooter.stopShooter();
        }

        if (robotWithFlickerShooter.getGamepad1().dpad_right){
            servController.setDegrees(-180);
        } else if (robotWithFlickerShooter.getGamepad1().dpad_left){
            servController.setDegrees(180);
        }
    }
}
