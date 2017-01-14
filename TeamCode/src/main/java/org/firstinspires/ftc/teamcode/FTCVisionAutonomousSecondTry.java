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
@Autonomous(name = "BitterBasesBlueAutonOp(FTCVision2ndtryBLUE)")
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
        servoControllerLib = new ServoControllerLib(hardwareMap.servo.get("btnServo"),ServoControllerLib.SERVOLEFT);
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
        telemetry.addData("Action","FWD 10CM");
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNTOPOSITION();
        telemetry.addData("running", "fwd 10 cm");
        robot.getBLM().setTargetPosition((int)(5.0*TICKSPERCENTIMETER));
        robot.getBRM().setTargetPosition((int)(5.0*TICKSPERCENTIMETER));
        robot.getBLM().setPower(.75);
        robot.getBRM().setPower(0.75);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition()<5.0*TICKSPERCENTIMETER){delay(1);};

        //Turn 35
        telemetry.addData("Action","Turn35");
        int ticksToGo = (int) (Math.PI*2.0*14.0*2.54/360.0*35.0*TICKSPERCENTIMETER);
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNTOPOSITION();
        robot.getBLM().setTargetPosition(ticksToGo);
        robot.getBLM().setPower(0.3);
        robot.getBRM().setPower(0.0);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition() < ticksToGo&&robot.getBLM().getCurrentPosition() < 100000){delay(1);}
        robot.getBLM().setPower(0.0);

        final double EOPDWHITELINELIGHTLEVEL = 0.15;//TODO Karim make sure this is right too
        //Go straight till EOPD
        telemetry.addData("Action", "Go straight till line");
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNWITHENCODERS();
        robot.getBLM().setPower(.3);
        robot.getBRM().setPower(.3);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition() < 100000&& ods.getLightDetected() < EOPDWHITELINELIGHTLEVEL){delay(1);}
        robot.getBLM().setPower(0.0);
        robot.getBRM().setPower(0.0);

        //Go back to make room
        telemetry.addData("Action","GoBackToMakeRoom");
        robot.getBLM().setDirection(DcMotorSimple.Direction.FORWARD);
        robot.getBRM().setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Action","SetMotorDirection");
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNTOPOSITION();
        telemetry.addData("Action","setRuntoPos");

        robot.getBLM().setTargetPosition((int)(5.0*TICKSPERCENTIMETER));
        robot.getBRM().setTargetPosition((int)(5.0*TICKSPERCENTIMETER));
        telemetry.addData("Action","setTargetPos");
        robot.getBLM().setPower(.75);
        robot.getBRM().setPower(0.75);
        telemetry.addData("Action","setPower");
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition()<5.0*TICKSPERCENTIMETER&&robot.getBLM().getCurrentPosition() < 100000){delay(1);};
        telemetry.addData("Action","DonewithWhileLoop");
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        robot.getBRM().setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Action","Normalized motor directions");

        //Turn 65
        //TODO Karim fiddle with this to make sure it faces the beacon head on
        ticksToGo = (int) (Math.PI*2.0*14.0*2.54/360.0*65.0*TICKSPERCENTIMETER);
        telemetry.addData("Action","Turn65");
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNTOPOSITION();
        robot.getBLM().setTargetPosition(ticksToGo);
        robot.getBLM().setPower(0.3);
        robot.getBRM().setPower(0.0);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition() < ticksToGo&&robot.getBLM().getCurrentPosition() < 100000){delay(1);}
        robot.getBLM().setPower(0.0);

        //Backup so camera can see whole beacon
        robot.getBLM().setDirection(DcMotorSimple.Direction.FORWARD);
        robot.getBRM().setDirection(DcMotorSimple.Direction.REVERSE);

        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNTOPOSITION();


        robot.getBLM().setTargetPosition((int)(30.0*TICKSPERCENTIMETER));
        robot.getBRM().setTargetPosition((int)(30.0*TICKSPERCENTIMETER));
        robot.getBLM().setPower(.75);
        robot.getBRM().setPower(0.75);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition()<30.0*TICKSPERCENTIMETER){delay(1);};

        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        robot.getBRM().setDirection(DcMotorSimple.Direction.FORWARD);
        //SHOULD BE FACONG BEACON

        /*End Manuver*/
        //BEGIN read beacon
        telemetry.addData("Action","Begin read beacon");
        int redBlue = 0;
        int blueRed = 0;
        for(int i=0; i < 5; i++){ //Purposefully not even number
            if (currentColorOrder.equals("red, blue")){
                redBlue++;
                telemetry.addData("redBlue",redBlue);
            }else if (currentColorOrder.equals("blue, red")){
                blueRed++;
                telemetry.addData("blueRed",blueRed);
            }else {
                i--;
                telemetry.addData("???, ???","??????");
            }
            delay(50); //Let vision process a new frame not get same info
        }
        if (redBlue > blueRed){
            //Press Right side b/c we are blue
            telemetry.addData("Analysis","Push right");
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            //Turn Servo
            //Drive Forwards to press with lower power, keep pushing for some time
            autonomousDrive.resetEncoders();
            autonomousDrive.setRUNWITHENCODERS();
            robot.getBLM().setPower(0.2);
            robot.getBRM().setPower(0.2);
            delay(3000);
            robot.getBLM().setPower(0.0);
            robot.getBRM().setPower(0.0);
        }else{
            //Press Left side b/c we are blue
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);//Turn Servo
            telemetry.addData("Analysis","Push left");
            //Drive Forwards to press with lower power, keep pushing for some time
            autonomousDrive.resetEncoders();
            autonomousDrive.setRUNWITHENCODERS();
            robot.getBLM().setPower(0.2);
            robot.getBRM().setPower(0.2);
            delay(3000);
            robot.getBLM().setPower(0.0);
            robot.getBRM().setPower(0.0);
        }

        //Backup, go to other beacon

        //Backup
        robot.getBLM().setDirection(DcMotorSimple.Direction.FORWARD);
        robot.getBRM().setDirection(DcMotorSimple.Direction.REVERSE);

        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNTOPOSITION();


        robot.getBLM().setTargetPosition((int)(30.0*TICKSPERCENTIMETER));
        robot.getBRM().setTargetPosition((int)(30.0*TICKSPERCENTIMETER));
        robot.getBLM().setPower(.75);
        robot.getBRM().setPower(0.75);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition()<30.0*TICKSPERCENTIMETER){delay(1);};

        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        robot.getBRM().setDirection(DcMotorSimple.Direction.FORWARD);

        //Turn 130
        //TODO Karim fiddle with this to make sure it faces the beacon head on
        ticksToGo = (int) (Math.PI*2.0*14.0*2.54/360.0*110.0*TICKSPERCENTIMETER);
        telemetry.addData("Action","Turn65");
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNTOPOSITION();
        robot.getBRM().setTargetPosition(ticksToGo);
        robot.getBRM().setPower(0.3);
        robot.getBLM().setPower(0.0);
        while(opModeIsActive()&&robot.getBRM().getCurrentPosition() < ticksToGo&&robot.getBRM().getCurrentPosition() < 100000){delay(1);}
        robot.getBLM().setPower(0.0);

        //Go straight till hit line
        //Go straight till EOPD
        telemetry.addData("Action", "Go straight");
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNWITHENCODERS();
        robot.getBLM().setPower(.3);
        robot.getBRM().setPower(.3);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition() < 100000&& ods.getLightDetected() < EOPDWHITELINELIGHTLEVEL){delay(1);}
        robot.getBLM().setPower(0.0);
        robot.getBRM().setPower(0.0);


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
