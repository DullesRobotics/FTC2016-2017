package com.dullesrobotics.ftc.archive;

import android.graphics.Bitmap;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Matrix34F;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

import java.util.Arrays;

/**
 * Created by kk200 on 11/5/2016.
 */

@Disabled
@Autonomous(name = "VuforiaOp")
public class VuforiaOp extends LinearVisionOpMode {
    long startTime;
    RobotWithFlickerShooter robot;
    ArcadeDrive ArcDrive;
    private ElapsedTime runtime = new ElapsedTime();
    String currentColorOrder = "???, ???";
    //Frame counter
    int frameCount = 0;
    ServoControllerLib servLib = new ServoControllerLib(hardwareMap.servo.get("BtnServo"));

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



        DcMotor driveR = hardwareMap.dcMotor.get("BRM");
        driveR.setDirection(DcMotorSimple.Direction.FORWARD);
        driveR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotor driveL = hardwareMap.dcMotor.get("BLM");
        driveL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveL.setDirection(DcMotorSimple.Direction.REVERSE);



        //FTCVISION INIT

        //BELOW is for FTCVISION
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

        driveL.setTargetPosition((int) (11.5*2.54/(3.62*3.14159)*1440.4/8.0));

        //Start Run
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ATUnO23/////AAAAGfN3NJaVtUVwqWhgjFxyO2h0TDN/26T4WFlJOjz7YbOqBoP+h6dhU77AmHvf4tDJMMLZAu602Z1jnohtpRmTDbVkZuQ3B1Ym/r9DEKBMT3zEbYALAdIBgF/a9rNGhxXDSJF9FSPxrfYaVrfLneryTb/DMbsH06UvKFYM6sZtaDaxHS1ZBX5LDMiUEHcmbXoGXpvjD50C3DZVgOiWBIlPz4lydCE9HiEWTm7z7O929fVMJjBwAu1xL1AWHeYw0mn4Ngyi82SefCTKBdcVz/JXtmWo721sn08pZPcpfPOpeJ2YHGw0uZPF+d9AQXJ8oVItZOtZG45yDLAaPnQnxYqHtJRgSj/40KqkuRSWkLW0R03A";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizerImplSubclass vuforia = new VuforiaLocalizerImplSubclass(params);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4); //Kinda self explanatory

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();

        beacons.activate();

        driveL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveL.setPower(0.2);
        driveR.setPower(0.2);


        while(opModeIsActive() && wheels.getRawPose() == null) {
            //idle();
            waitOneFullHardwareCycle();
        }

        driveL.setPower(0);
        driveR.setPower(0);

        //Analyze beacon here

        VectorF angles = anglesFromTarget(wheels);
        VectorF trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) -90, new VectorF(500,0,0));

        if(trans.get(0) > 0){
            driveL.setPower(0.02);
            driveR.setPower(-0.02);
        } else{
            driveL.setPower(-0.02);
            driveR.setPower(0.02);
        }

        do{
            if(wheels.getPose() != null){
                trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) -90, new VectorF(500,0,0));
            }
            //idle();
            waitOneFullHardwareCycle();

        }while (opModeIsActive() && Math.abs(trans.get(0)) > 30);

        driveL.setPower(0);
        driveR.setPower(0);




        driveL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveR.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        driveL.setTargetPosition((int) (driveL.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 150 / 113.668 * 1478.4 ))));
        driveR.setTargetPosition((int) (driveL.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 150 / 113.668 * 1478.4 ))));

        //THIE NUMBER 150 will be different for ou rrobot

        driveL.setPower(0.3);
        driveR.setPower(0.3);

        while(opModeIsActive() && driveL.isBusy() && driveR.isBusy()){
           // idle();
            waitOneFullHardwareCycle();
        }


        driveL.setPower(0);
        driveR.setPower(0);


        driveL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && (wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0)) > 10)) {
            if (wheels.getPose() != null) {
                    if(wheels.getPose().getTranslation().get(0) > 0 ){
                        driveL.setPower(-0.3);
                        driveR.setPower(0.3);
                    } else {
                        driveL.setPower(0.3);
                        driveR.setPower(-0.3);
                    }

            } else {
                driveL.setPower(-0.3);
                driveR.setPower(0.3);
            }
        }

        driveL.setPower(0);
        driveR.setPower(0);



        while(opModeIsActive()){
            if(vuforia.rgb != null){
                    Bitmap bn = Bitmap.createBitmap(vuforia.rgb.getWidth(),vuforia.rgb.getHeight(),Bitmap.Config.RGB_565);
                    bn.copyPixelsFromBuffer(vuforia.rgb.getPixels());

            }


             for (VuforiaTrackable b : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) b.getListener()).getRawPose();
                if (pose != null){
                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(),0,12);
                    rawPose.setData(poseData);

                    Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(),rawPose, new Vec3F(-127,92,0));
                    Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(),rawPose, new Vec3F(127,92,0));
                    Vec2F lowerRight = Tool.projectPoint(vuforia.getCameraCalibration(),rawPose, new Vec3F(127,-92,0));
                    Vec2F lowerLeft = Tool.projectPoint(vuforia.getCameraCalibration(),rawPose, new Vec3F(-127,-92,0));

                }
            }
            telemetry.update();






        }


        //Lined up with beacon
        driveL.setPower(-0.2);
        driveR.setPower(-0.2);//go back
        delay(1000);
        driveL.setPower(0.0);
        driveR.setPower(0.0);



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
            servLib.setDegrees(180);
            //Turn Servo
            //Drive Forwards to press with lower power, keep pushing for some time
        }else{
            //Press Left side b/c we are blue

            //Turn Servo
            //Drive Forwards to press with lower power, keep pushing for some time
        }




    }
    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }
    public void delay(long millis){
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() < start+millis){}
    }

}
