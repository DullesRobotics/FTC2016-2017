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
public class AutonomousBlueV4 extends LinearVisionOpMode {
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
        autonomousDrive = new AutonomousDrive(this,robot,hardwareMap.opticalDistanceSensor.get("EOPD"));
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
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();


        waitForStart(); //Wait for START Button Press on DS

        //START


        /*End Manuver*/
        //BEGIN read beacon
        telemetry.addData("Action","Begin read beacon");
        int redBlue = 0;
        int blueRed = 0;
        for(int i=0; i < 5 && opModeIsActive(); i++){ //Purposefully not even number
            if (beacon.getAnalysis().getColorString().equals("red, blue")){
                redBlue++;
                telemetry.addData("redBlue",redBlue);
            }else if (beacon.getAnalysis().getColorString().equals("blue, red")){
                blueRed++;
                telemetry.addData("blueRed",blueRed);
            }else {
                i--;
                telemetry.addData("???, ???",beacon.getAnalysis());
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


        //2nd Beacon
        redBlue = 0;
        blueRed = 0;

        for(int i=0; i < 5; i++){ //Purposefully not even number
            if (beacon.getAnalysis().getColorString().equals("red, blue")){
                redBlue++;
            }else if (beacon.getAnalysis().getColorString().equals("blue, red")){
                blueRed++;
            }else {
                i--;
            }
            delay(50); //Let vision process a new frame not get same info
        }
        if (redBlue > blueRed){
            //Press Right side b/c we are blue
            servoControllerLib.setDegrees(180);
            autonomousDrive.resetEncoders();
            autonomousDrive.setRUNWITHENCODERS();
            robot.getBLM().setPower(0.2);
            robot.getBRM().setPower(0.2);
            delay(3000);
            robot.getBLM().setPower(0.0);
            robot.getBRM().setPower(0.0);
            //Turn Servo
            //Drive Forwards to press with lower power, keep pushing for some time
        }else{
            //Press Left side b/c we are blue
            servoControllerLib.setDegrees(0);

            //Turn Servo
            //Drive Forwards to press with lower power, keep pushing for some time
            autonomousDrive.resetEncoders();
            autonomousDrive.setRUNWITHENCODERS();
            robot.getBLM().setPower(0.2);
            robot.getBRM().setPower(0.2);
            delay(3000);
            robot.getBLM().setPower(0.0);
            robot.getBRM().setPower(0.0);
        }


        //Turn around to face center, ram ball and park partially

    }
}
