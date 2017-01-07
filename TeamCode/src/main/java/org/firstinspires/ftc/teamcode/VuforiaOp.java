package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;
import com.vuforia.Matrix34F;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;
import java.util.Vector;

/**
 * Created by kk200 on 11/5/2016.
 */

@TeleOp(name = "Vuforia Test")
public class VuforiaOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor driveR = hardwareMap.dcMotor.get("BRM");
        driveR.setDirection(DcMotorSimple.Direction.REVERSE);
        driveR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotor driveL = hardwareMap.dcMotor.get("BLM");
        driveL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



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
            idle();
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
            idle();

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
            idle();
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

}
