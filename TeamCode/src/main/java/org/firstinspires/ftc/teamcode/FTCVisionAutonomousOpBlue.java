package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import java.util.ArrayList;

/**
 * Linear Vision Sample
 * <p/>
 * Use this in a typical linear op mode. A LinearVisionOpMode allows using
 * Vision Extensions, which do a lot of processing for you. Just enable the extension
 * and set its options to your preference!
 * <p/>
 * Please note that the LinearVisionOpMode is specially designed to target a particular
 * version of the FTC Robot Controller app. Changes to the app may break the LinearVisionOpMode.
 * Should this happen, open up an issue on GitHub. :)
 */
@Autonomous(name = "FTCVISION Autonomous")
public class FTCVisionAutonomousOpBlue extends LinearVisionOpMode {
    //Our Variables
    final double ENCODERTICKSPERREVOLUTION = 1478.4;
    final double CIRCUMFERENCEOFWHEELCENTIMETERS = Math.PI*9.6;//ModernRobotics has generously not provided us with a CAD file for the wheel so let's use 96mm diam until we get an actual measurement
    long startTime;
    RobotWithFlickerShooter robot;
    ArcadeDrive ArcDrive;
    private ElapsedTime runtime = new ElapsedTime();

    //BELOW is for FTCVISION
    String currentColorOrder = "???, ???";
    //Frame counter
    int frameCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();
        //Below not camera stuff
        startTime = System.currentTimeMillis();
        //Initialize Variables
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),gamepad1,hardwareMap.dcMotor.get("flickerShooter"));
        //Set to Arcade Drive (Autonomous Arcade Drive)
        ArcDrive = new ArcadeDrive(robot);
        robot.setDriveTrain(ArcDrive);
        //end not camera stuff ^^^^^^^^
        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.PRIMARY);

        /**
         * Set the frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
        this.setFrameSize(new Size(900, 900));

        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /**
         * Set analysis boundary
         * You should comment this to use the entire screen and uncomment only if
         * you want faster analysis at the cost of not using the entire frame.
         * This is also particularly useful if you know approximately where the beacon is
         * as this will eliminate parts of the frame which may cause problems
         * This will not work on some methods, such as COMPLEX
         **/
        //beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));

        /**
         * Set the rotation parameters of the screen
         * If colors are being flipped or output appears consistently incorrect, try changing these.
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * It's a good idea to disable global auto rotate in Android settings. You can do this
         * by calling disableAutoRotate() or enableAutoRotate().
         *
         * It's also a good idea to force the phone into a specific orientation (or auto rotate) by
         * calling either setActivityOrientationAutoRotate() or setActivityOrientationFixed(). If
         * you don't, the camera reader may have problems reading the current orientation.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        //Wait for the match to begin
        waitForStart();

        /*  Manuver so robot is facing beacon */

        /*
        //Shoot ONE Ball
        robot.turnBackwards();
        delay(5000);
        robot.stopShooter();
        */

        //LOOK AT LINEFOLLOWER CLASS IN LIBRARIES

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
            //Press Right side b/c we are blue

            //Turn Servo
            //Drive Forwards to press with lower power, keep pushing for some time
        }else{
            //Press Left side b/c we are blue

            //Turn Servo
            //Drive Forwards to press with lower power, keep pushing for some time
        }


        //Turn around to face center, ram ball and park partially
















        //Main loop
        //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
        //This loop will exit once the opmode is closed
        while (opModeIsActive()) {
            //Log a few things
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Frame Counter", frameCount);
            currentColorOrder = beacon.getAnalysis().getColorString();
            //You can access the most recent frame data and modify it here using getFrameRgba() or getFrameGray()
            //Vision will run asynchronously (parallel) to any user code so your programs won't hang
            //You can use hasNewFrame() to test whether vision processed a new frame
            //Once you copy the frame, discard it immediately with discardFrame()
            if (hasNewFrame()) {
                //Get the frame
                Mat rgba = getFrameRgba();
                Mat gray = getFrameGray();
                /*
                String test = beacon.getAnalysis().getColorString();
                telemetry.addData("TestOutput",test);
                */
                //Discard the current frame to allow for the next one to render
                discardFrame();

                //Do all of your custom frame processing here
                //For this demo, let's just add to a frame counter
                frameCount++;
            }

            //Wait for a hardware cycle to allow other processes to run
            waitOneFullHardwareCycle();
        }
    }

    /* BEGIN COPY PASTED METHODS */
    public void delay(long millis){
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() < start+millis){}
    }
    public double secsSinceStart(){
        return ((Number)(System.currentTimeMillis()-startTime)).doubleValue()/(1000.0);
    }
    public void stopAndResetEncoders() throws InterruptedException {
        robot.getBLM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitOneFullHardwareCycle();
    }
    public int distToEncoderTicks(double dist){
        return (int)(dist/CIRCUMFERENCEOFWHEELCENTIMETERS*ENCODERTICKSPERREVOLUTION);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder // CHECK IF THIS IS CORRECT
        final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP //CHECK IF THIS IS CORRECT
        final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference  // CHECK IF THIS IS CORRECT
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.getBLM().getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.getBRM().getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.getBLM().setTargetPosition(newLeftTarget);
            robot.getBRM().setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.getBLM().setPower(Math.abs(speed));
            robot.getBRM().setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.getBLM().isBusy() && robot.getBRM().isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.getBLM().getCurrentPosition(),
                        robot.getBRM().getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.getBLM().setPower(0);
            robot.getBRM().setPower(0);

            // Turn off RUN_TO_POSITION
            robot.getBLM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getBRM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
