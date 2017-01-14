package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.AutonomousDrive;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

import static com.dullesrobotics.ftc.libraries.commonMethods.delay;

/**
 * Created by Kenneth on 1/7/2017.
 */
@Autonomous(name = "UseThisBlue")
public class FTCVisionAutonomousSecondTry extends LinearVisionOpMode {
    final static double ENCODERTICKSPERREVOLUTION = 1478.4;
    final static double CIRCUMFERENCEOFWHEELCENTIMETERS = Math.PI*9.6;
    final static double TICKSPERCENTIMETER = ENCODERTICKSPERREVOLUTION/CIRCUMFERENCEOFWHEELCENTIMETERS;
    private ElapsedTime runtime = new ElapsedTime();
    RobotWithFlickerShooter robot;
    AutonomousDrive autonomousDrive;
    String currentColorOrder = "???, ???";
    int sleepTime = 0;
    ServoControllerLib servoControllerLib;
    OpticalDistanceSensor ods;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();

        //Initialize Robot
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),gamepad1,hardwareMap.dcMotor.get("flickerShooter"));
        autonomousDrive = new AutonomousDrive(robot,hardwareMap.opticalDistanceSensor.get("EOPD"));
        servoControllerLib = new ServoControllerLib(hardwareMap.servo.get("btnServo"),180);
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        ods = hardwareMap.opticalDistanceSensor.get("EOPD");
        //Sets Up Camera
        //initializes camera
        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));
        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);
        enableExtension(Extensions.CAMERA_CONTROL);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

/*
        //Allow Setting a time delay
        final int programedDelay = 10;
        final boolean overrideControls = false;
        final int MAXSLEEPTIME = 20;

        boolean prevStateBtnA = false;
        boolean prevStateBtnB = false;

        //You have 10 seconds to set the delay
        while(runtime.seconds() < 10 && opModeIsActive()) {
            if(gamepad1.a){
                sleepTime+=5;
                if (sleepTime > MAXSLEEPTIME){
                    sleepTime = MAXSLEEPTIME;
                }
            } else if (gamepad1.b){
                sleepTime-=5;
                if(sleepTime < 0){
                    sleepTime = 0;
                }
            }


            prevStateBtnA = gamepad1.a;
            prevStateBtnB = gamepad1.b;
        }
*/
        waitForStart(); //Wait for START Button Press on DS
        //delay(sleepTime*1000);

        //START

        //Drive forwards until ODS is triggered by white tape
        //autonomousDrive.driveStraightForSetTime(2,.75);
        //FWD 10cm
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNTOPOSITION();
        telemetry.addData("running", "fwd 10 cm");
        robot.getBLM().setTargetPosition((int)(10.0*TICKSPERCENTIMETER));
        robot.getBRM().setTargetPosition((int)(10.0*TICKSPERCENTIMETER));
        robot.getBLM().setPower(.75);
        robot.getBRM().setPower(0.75);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition()<10*TICKSPERCENTIMETER){};

        //Turn 40
        int ticksToGo = (int) (Math.PI*2.0*14.0*2.54/360.0*40.0*TICKSPERCENTIMETER);
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNTOPOSITION();
        robot.getBLM().setTargetPosition(ticksToGo);
        robot.getBLM().setPower(0.3);
        robot.getBRM().setPower(0.0);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition() < ticksToGo&&robot.getBLM().getCurrentPosition() < 100000){}
        robot.getBLM().setPower(0.0);


        //Go straight till EOPD
        telemetry.addData("Action", "Go straight 30cm");
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNWITHENCODERS();
        robot.getBLM().setPower(.3);
        robot.getBRM().setPower(.3);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition() < 100000&& ods.getLightDetected() < 0.15){}
        robot.getBLM().setPower(0.0);
        robot.getBRM().setPower(0.0);

        //Turn 50
        ticksToGo = (int) (Math.PI*2.0*14.0*2.54/360.0*50.0*TICKSPERCENTIMETER);
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNTOPOSITION();
        robot.getBLM().setTargetPosition(ticksToGo);
        robot.getBLM().setPower(0.3);
        robot.getBRM().setPower(0.0);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition() < ticksToGo&&robot.getBLM().getCurrentPosition() < 100000){}
        robot.getBLM().setPower(0.0);



        //Begin Turn
        autonomousDrive.resetEncoders();


        telemetry.addData("Action", "pt turn 30cm");
        autonomousDrive.pointTurn(45.0);
        autonomousDrive.driveStraightTillEOPD(300.0,0.5);//Drive fwd and stop at line
        autonomousDrive.pointTurn(45.0);//face beacon

        /*End Manuver*/
        int redBlue = 0;
        int blueRed = 0;
        for(int i=0; i < 19; i++){ //Purposefully not even number
            if (currentColorOrder.equals("red, blue")){
                redBlue++;
            }else if (currentColorOrder.equals("blue, red")){
                blueRed++;
            }else if (currentColorOrder.equals("???, ???")){
                i--;
            }else{
                throw new Error("currentColorOrder invalid String");
            }
            delay(50); //Let vision process a new frame not get same info
        }
        if (redBlue > blueRed){
            //Press Right side b/c we are blue2

            //Turn Servo
            //Drive Forwards to press with lower power, keep pushing for some time
        }else{
            //Press Left side b/c we are blue

            //Turn Servo
            //Drive Forwards to press with lower power, keep pushing for some time
        }

        //Backup, go to other beacon

        //2nd Beacon
        redBlue = 0;
        blueRed = 0;

        for(int i=0; i < 5; i++){ //Purposefully not even number
            if (currentColorOrder.equals("red, blue")){
                redBlue++;
            }else if (currentColorOrder.equals("blue, red")){
                blueRed++;
            }else {
                i--;
            }
            delay(50); //Let vision process a new frame not get same info
        }
        if (redBlue > blueRed){
            //Press Right side b/c we are blue
            servoControllerLib.setDegrees(180);
            autonomousDrive.driveStraightForSetTime(1.5,0.2);
            
            //Turn Servo
            //Drive Forwards to press with lower power, keep pushing for some time
        }else{
            //Press Left side b/c we are blue
            servoControllerLib.setDegrees(0);
            autonomousDrive.driveStraightForSetTime(1.5,0.2);
            //Turn Servo
            //Drive Forwards to press with lower power, keep pushing for some time
        }


        //Turn around to face center, ram ball and park partially

    }
}
