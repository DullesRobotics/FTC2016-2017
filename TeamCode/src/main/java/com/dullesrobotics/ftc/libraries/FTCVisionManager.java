package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

import static com.dullesrobotics.ftc.libraries.commonMethods.delay;

/**
 * Created by Kenneth on 1/15/2017.
 */

public class FTCVisionManager {
    LinearVisionOpMode opMode;
    Beacon.AnalysisMethod analysisMethod;
    public ElapsedTime runtime;

    public FTCVisionManager(LinearVisionOpMode op, Beacon.AnalysisMethod aM){
        opMode = op;
        analysisMethod = aM;
        runtime = new ElapsedTime();
    }
    public void initFTCVision() throws InterruptedException {
        opMode.waitForVisionStart();
        opMode.setCamera(Cameras.PRIMARY);
        opMode.setFrameSize(new Size(900, 900));
        opMode.enableExtension(VisionOpMode.Extensions.BEACON);
        opMode.enableExtension(VisionOpMode.Extensions.ROTATION);
        opMode.enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL);
        opMode.beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);
        opMode.beacon.setColorToleranceRed(0);
        opMode.beacon.setColorToleranceBlue(0);
        opMode.rotation.setIsUsingSecondaryCamera(false);
        opMode.rotation.disableAutoRotate();
        opMode.rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
        opMode.cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        opMode.cameraControl.setAutoExposureCompensation();
    }

    /**
     *
     * @param times make sure this is an odd number to prevent a forced decision. A recommended value is between 3 and 11 - I like 5
     * @param timeoutS Set timeout - if processing is not done by this time, return best guess
     */
    public String readBeacon(int times,double timeoutS){
        int redBlue = 0;
        int blueRed = 0;
        runtime.reset();
        for(int i=0; i < 5 && opMode.opModeIsActive() && runtime.seconds() < timeoutS; i++){ //Purposefully not even number
            if (opMode.beacon.getAnalysis().getColorString().equals("red, blue")){
                redBlue++;
                opMode.telemetry.addData("redBlue",redBlue);
            }else if (opMode.beacon.getAnalysis().getColorString().equals("blue, red")){
                blueRed++;
                opMode.telemetry.addData("blueRed",blueRed);
            }else {
                i--;
                opMode.telemetry.addData("???, ???",opMode.beacon.getAnalysis());
            }
            delay(50); //Let vision process a new frame not get same info
        }
        if(redBlue > blueRed){
            return "redBlue";
        }else{
            return "blueRed";
        }
    }
}
