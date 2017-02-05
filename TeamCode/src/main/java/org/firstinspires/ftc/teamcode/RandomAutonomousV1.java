package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV2;
import com.dullesrobotics.ftc.libraries.BasicRobot;
import com.dullesrobotics.ftc.libraries.FTCVisionManager;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by adilras on 2/4/2017.
 */

public class RandomAutonomousV1 extends LinearOpMode {
    BasicRobot robot;
    final static double ENCODERTICKSPERREVOLUTION = 1478.4;
    final static double CIRCUMFERENCEOFWHEELCENTIMETERS = Math.PI*9.6;
    //final static double TICKSPERCENTIMETER = ENCODERTICKSPERREVOLUTION/CIRCUMFERENCEOFWHEELCENTIMETERS;
    final static double TICKSPERCENTIMETER = (5000.0/39.0/2.54);
    //final static double TICKSPERCENTIMETER = 109.36132983377077865266841644794;
    final static double DISTANCEBETWEENWHEELSINCHES = 14.0;
    final static double POINTTURNRADIUSCM = DISTANCEBETWEENWHEELSINCHES/(2.0*2.54);
    final static double SWINGTURNRADIUSCM = DISTANCEBETWEENWHEELSINCHES * 2.54;
    public final static double EOPDWHITELINELIGHTLEVEL = 0.05;
    //public final static double EOPDWHITELINELIGHTLEVEL = 0.084;
    private boolean isReversed = false;
    LinearVisionOpMode opMode;
    private ElapsedTime runtime = new ElapsedTime();
    String currentColorOrder = "???, ???";
    int sleepTime = 0;
    RobotWithFlickerShooter robotWithFlicker;
    AutonomousDriveClassV2 autonomousDrive;
    ServoControllerLib servoControllerLib;
    OpticalDistanceSensor ods;
    FTCVisionManager ftcVisionManager;
    final boolean DEBUG = true;
    final boolean bothBeacons = false;
    final boolean parkCorner = false;
    ServoControllerLib leftShooter,rightShooter;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
    }
    public void debug(int i) throws InterruptedException {
        String num = Integer.toString(i);
        if(DEBUG){
            opMode.telemetry.addData("AutonomousDrive flag",num);
            opMode.telemetry.addData("Status",opMode.opModeIsActive());
            opMode.telemetry.update();
            //delay(250);
            opMode.waitOneFullHardwareCycle();
            //delay(100);
            return;
        }else{
            return;
        }
    }
}
