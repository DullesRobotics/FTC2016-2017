package com.dullesrobotics.ftc.libraries;

import com.dullesrobotics.ftc.mods.SensorListener;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import java.util.concurrent.atomic.AtomicIntegerFieldUpdater;

import static com.dullesrobotics.ftc.libraries.commonMethods.delay;

/**
 * Created by Kenneth on 1/8/2017.
 * Modified by Karim
 */

public class AutonomousDriveClassV2 {
    //BasicRobot robot;
    AdvancedRobot robot;

    final static double ENCODERTICKSPERREVOLUTION = 1478.4;
    final static double CIRCUMFERENCEOFWHEELCENTIMETERS = Math.PI*9.6;
    //final static double TICKSPERCENTIMETER = ENCODERTICKSPERREVOLUTION/CIRCUMFERENCEOFWHEELCENTIMETERS;
    final static double TICKSPERCENTIMETER = (5000.0/39.0/2.54);
    //final static double TICKSPERCENTIMETER = 109.36132983377077865266841644794;
    final static double DISTANCEBETWEENWHEELSINCHES = 13.5;
    final static double POINTTURNRADIUSCM = DISTANCEBETWEENWHEELSINCHES/(2.0*2.54);
    final static double SWINGTURNRADIUSCM = DISTANCEBETWEENWHEELSINCHES * 2.54;
    public static double EOPDWHITELINELIGHTLEVEL = 0.125;
    //public final static double EOPDWHITELINELIGHTLEVEL = 0.084;
    private boolean isReversed = false;
    OpticalDistanceSensor ods;
    LinearVisionOpMode opMode;
    private ElapsedTime runtime;
    ServoControllerLib servoControllerLib;
    FTCVisionManager ftcVisionManager;

    //SensorListener sensorListener;
    //float heading;
/*
    public AutonomousDrive(BasicRobot r, SensorListener s) {
        robot = r;
        sensorListener = s;
    }
*/
    /*public AutonomousDriveClassV2(BasicRobot r) {
        robot = r;
    }*/

    public AutonomousDriveClassV2(AdvancedRobot r){ robot = r; }

    public AutonomousDriveClassV2(LinearVisionOpMode op, BasicRobot r, OpticalDistanceSensor o, ServoControllerLib servoLib, FTCVisionManager vision) {
        robot = (AdvancedRobot) r;
        ods = o;
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        opMode = op;
        runtime  = new ElapsedTime();
        servoControllerLib = servoLib;
        ftcVisionManager = vision;
    }



    public int getMaxCurrentPos() throws InterruptedException{
        return Math.max(robot.getBLM().getCurrentPosition(),robot.getBRM().getCurrentPosition());
    }

    public void setRUNWITHENCODERS() throws InterruptedException{
        robot.getBLM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setRUNWITHOUTENCODERS() throws InterruptedException{
        robot.getBLM().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setRUNTOPOSITION() throws InterruptedException{
        robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetEncoders() throws InterruptedException{
        robot.getBLM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) throws InterruptedException{
        robot.getBLM().setZeroPowerBehavior(zeroPowerBehavior);
        robot.getBRM().setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void switchDirection() throws InterruptedException{
        if(isReversed){
            //Unreverse
            robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
            robot.getBRM().setDirection(DcMotorSimple.Direction.FORWARD);
            isReversed = false;
        }else{
            robot.getBLM().setDirection(DcMotorSimple.Direction.FORWARD);
            robot.getBRM().setDirection(DcMotorSimple.Direction.REVERSE);
            isReversed = true;
        }
    }
    public void setDirectionReverse() throws InterruptedException{
        robot.getBLM().setDirection(DcMotorSimple.Direction.FORWARD);
        robot.getBRM().setDirection(DcMotorSimple.Direction.REVERSE);
        isReversed = true;
    }
    public void setDirectionNotReversed() throws InterruptedException{
        //Unreverse
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        robot.getBRM().setDirection(DcMotorSimple.Direction.FORWARD);
        isReversed = false;
    }
    public void driveTillLine(double power,double timeoutS) throws InterruptedException{
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        setRUNWITHENCODERS();
        setDirectionNotReversed();
        robot.getBRM().setPower(power);
        robot.getBLM().setPower(power);
        while(!isOnLine() && timer.seconds() < timeoutS && opMode.opModeIsActive()){
            opMode.waitOneFullHardwareCycle();
        }
        resetAll();
    }

     public void setEOPDWHITELINELIGHTLEVEL(double multiplier,int nums){
         double values = 0;
         for(int i=0;i<nums;i++){
             values += ods.getLightDetected();
         }
         double avg = values/((double)(nums));
         EOPDWHITELINELIGHTLEVEL = avg*multiplier;
         opMode.telemetry.addData("EOPDLEVEL",EOPDWHITELINELIGHTLEVEL);
         opMode.telemetry.update();
     }
    public void followLine(double power, double timeOutS) throws InterruptedException {
        setDirectionReverse();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        setRUNWITHENCODERS();

        while(timer.seconds() <= timeOutS && opMode.opModeIsActive()){
            double reflectance = ods.getLightDetected();

            if(reflectance >= EOPDWHITELINELIGHTLEVEL){
                robot.getBRM().setPower(-power);
                robot.getBLM().setPower(0);
            }
            else{
                robot.getBLM().setPower(-power);
                robot.getBRM().setPower(0);

            }

            opMode.telemetry.addData("Reflectance", reflectance);
            opMode.telemetry.update();
            opMode.waitOneFullHardwareCycle();

        }
        resetAll();

    }
    public void followLine(double power, double timeOutS, double distanceInches) throws InterruptedException {
        setDirectionReverse();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime restrictTimer = new ElapsedTime();
        restrictTimer.reset();
        timer.reset();
        resetEncoders();
        setRUNWITHENCODERS();
        boolean change = false;

        int initalCountLeft = robot.getBLM().getCurrentPosition();
        int initalCountRight = robot.getBRM().getCurrentPosition();

        while((timer.seconds() <= timeOutS && opMode.opModeIsActive()) && (robot.getBLM().getCurrentPosition()-initalCountLeft <= (distanceInches*ENCODERTICKSPERREVOLUTION)) && (robot.getBRM().getCurrentPosition()-initalCountRight <= (distanceInches*ENCODERTICKSPERREVOLUTION))){
            double reflectance = ods.getLightDetected();
            /*if (restrictTimer.seconds() > 3){
                restrictTimer.reset();
                if (reflectance >= EOPDWHITELINELIGHTLEVEL){
                    reflectance = EOPDWHITELINELIGHTLEVEL - 1;
                } else {
                    reflectance = EOPDWHITELINELIGHTLEVEL + 1;
                }
            }*/
            if(reflectance >= EOPDWHITELINELIGHTLEVEL) { //|| change){
                /*if (change){
                    robot.getBLM().setPower(-power);
                    robot.getBRM().setPower(0);
                    delay(2500);
                    change = false;
                    restrictTimer.reset();
                }*/
                robot.getBLM().setPower(-power);
                robot.getBRM().setPower(0);
            }
            else{
                robot.getBRM().setPower(-power);
                robot.getBLM().setPower(0);

            }

            /*if (restrictTimer.seconds() > 2 || !change) {
                change = true;
            }*/

            if (isOnLine()){
                change = false;
            }
            opMode.telemetry.addData("Reflectance", reflectance);
            opMode.telemetry.update();
            opMode.waitOneFullHardwareCycle();

        }
        robot.getBLM().setPower(0);
        robot.getBRM().setPower(0);
        resetAll();

    }

    public void followLineBackwards(double power, double timeOutS, double distanceInches) throws InterruptedException {
        setDirectionNotReversed();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        resetEncoders();
        setRUNWITHENCODERS();

        int initalCountLeft = robot.getBLM().getCurrentPosition();
        int initalCountRight = robot.getBRM().getCurrentPosition();

        while(timer.seconds() <= timeOutS && opMode.opModeIsActive() && (robot.getBLM().getCurrentPosition()-initalCountLeft <= (distanceInches*ENCODERTICKSPERREVOLUTION)) && (robot.getBRM().getCurrentPosition()-initalCountRight <= (distanceInches*ENCODERTICKSPERREVOLUTION))){
            double reflectance = ods.getLightDetected();

            if(reflectance >= EOPDWHITELINELIGHTLEVEL){
                robot.getBRM().setPower(-power);
                robot.getBLM().setPower(0);
            }
            else{
                robot.getBLM().setPower(-power);
                robot.getBRM().setPower(0);

            }

            opMode.telemetry.addData("Reflectance", reflectance);


        }
        setDirectionReverse();

    }


    public void pointTurn(double power,double deg,double timeoutS) throws InterruptedException {
        if(deg == 0.0){
            return;
        }
        ////double leftDistCM = (2.0 * Math.PI * POINTTURNRADIUSCM * deg)/ 360.0;
        double leftDistCM = ((((deg/360) * 2.0 * Math.PI)* 7.0) - 10) + 2.54;
        //mess with later to get calibration right.

        //double leftDistCM = ((((deg/360) * 2.0 * Math.PI)* POINTTURNRADIUSCM)) + 2.54;
        double rightDistCM = -leftDistCM;
        opMode.telemetry.addData("leftDistCM",leftDistCM);
        opMode.telemetry.update();
        encoderDrive(power,leftDistCM,rightDistCM,timeoutS);
    }

    public void swingTurn(double power,double deg,double timeoutS) throws InterruptedException {
        //degreee greater than 0 is right
        //degree less than 0 is left
        if(deg > 0){
            double leftDistCM = 2.0 * Math.PI * SWINGTURNRADIUSCM * deg / 360.0;
            double rightDistCM = 0;
            resetEncoders();
            robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getBRM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getBLM().setTargetPosition((int) (TICKSPERCENTIMETER*leftDistCM));
            robot.getBLM().setPower(power);
            robot.getBRM().setPower(0.0);
            while (robot.getBLM().isBusy() && opMode.opModeIsActive()){delay(1); opMode.waitOneFullHardwareCycle();}
        }else if (deg < 0){
            deg = Math.abs(deg);
            double leftDistCM = 0;
            double rightDistCM = 2.0 * Math.PI * SWINGTURNRADIUSCM * deg / 360.0;
            resetEncoders();
            robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getBLM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getBRM().setTargetPosition((int) (TICKSPERCENTIMETER*leftDistCM));
            robot.getBRM().setPower(power);
            robot.getBLM().setPower(0.0);
            while(robot.getBRM().isBusy() && opMode.opModeIsActive()){delay(1);opMode.waitOneFullHardwareCycle();}
        }
    }

    public boolean encoderDrive  (double speed,
                                  double leftCM, double rightCM,
                                  double timeoutS) throws InterruptedException {
        //rightCM *= 2;
        //leftCM = -leftCM;
        //rightCM = -rightCM;
        int newLeftTarget;
        int newRightTarget;
        //robot.getBLM().set
        this.resetEncoders();
        runtime.reset();
        boolean hitTimeOut = false;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            debug(1);
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.getBLM().getCurrentPosition() + (int)(leftCM * TICKSPERCENTIMETER);
            newRightTarget = robot.getBRM().getCurrentPosition() + (int)(rightCM * TICKSPERCENTIMETER);
            robot.getBLM().setTargetPosition(newLeftTarget);
            robot.getBRM().setTargetPosition(newRightTarget);
            debug(2);
            // Turn On RUN_TO_POSITION
            robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            debug(3);
            runtime.reset();
            robot.getBLM().setPower(Math.abs(speed));
            robot.getBRM().setPower(Math.abs(speed));
            debug(4);
            // keep looping while we are still active, and there is time left, and both motors are running.
            opMode.telemetry.addData("opModeisActive",opMode.opModeIsActive());
            opMode.telemetry.addData("runtime.seconds() < timeoutS",(runtime.seconds() < timeoutS));
            opMode.telemetry.addData("at least one motor Busy",(robot.getBLM().isBusy() || robot.getBRM().isBusy()));
            opMode.telemetry.update();
            while (opMode.opModeIsActive() && (robot.getBLM().isBusy() || robot.getBRM().isBusy() || runtime.seconds() < timeoutS)) {
                delay(1);
                // Display it for the driver.
                opMode.telemetry.addData("Targets",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                opMode.telemetry.addData("CurrentPos",  "Running at %7d :%7d",
                        robot.getBLM().getCurrentPosition(),
                        robot.getBRM().getCurrentPosition());
                opMode.telemetry.update();
                debug(5);
                if(runtime.seconds() > timeoutS){
                    hitTimeOut = true;
                    return hitTimeOut;
                }
                debug(6);
                if (!robot.getBLM().isBusy() && !robot.getBRM().isBusy()){
                    return false;
                }
                opMode.waitOneFullHardwareCycle();
            }
            debug(7);
            // Stop all motion;
            robot.getBLM().setPower(0);
            robot.getBRM().setPower(0);

            // Turn off RUN_TO_POSITION
            robot.getBLM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getBRM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //if (!opMode.opModeIsActive()){
            //opMode.stop();
            ///}

            //delay(250);   // optional pause after each move

        } else {
            debug(-1);
            //opMode.stop();
        }
        debug(10);
        resetAll();
        return hitTimeOut;
    }

    public boolean encoderDriveInches  (double speed,
                                         double leftIn, double rightIn,
                                         double timeoutS) throws InterruptedException {
        leftIn *= 2.54;
        rightIn *= 2.54;
        return encoderDrive(speed,leftIn,rightIn,timeoutS);
    }

    public boolean encoderDriveInches  (double speed,
                                        double dis,
                                        double timeoutS) throws InterruptedException {
        dis *= 2.54;
        return encoderDrive(speed,dis,dis,timeoutS);
    }

    public boolean isReversed(){
        return isReversed;
    }

    public void turnSetTime(double power, double seconds,boolean right) throws InterruptedException {
        setRUNWITHENCODERS();
        runtime.reset();
        if (right) {
            robot.getBLM().setPower(power);
            robot.getBRM().setPower(-power);
        } else {
            robot.getBLM().setPower(-power);
            robot.getBRM().setPower(power);
        }

        while (opMode.opModeIsActive()&&runtime.seconds()<seconds){
            opMode.telemetry.addData("Running for set time",seconds);
            opMode.telemetry.addData("Current time",runtime.seconds());
            opMode.telemetry.update();
            opMode.waitOneFullHardwareCycle();
        }

        robot.getBLM().setPower(0.0);
        robot.getBRM().setPower(0.0);
    }

    public void runForSetTime(double power, double seconds) throws InterruptedException {
        setRUNWITHENCODERS();
        runtime.reset();
        robot.getBLM().setPower(power);
        robot.getBRM().setPower(power);

        while (opMode.opModeIsActive()&&runtime.seconds()<seconds){
            opMode.telemetry.addData("Running for set time",seconds);
            opMode.telemetry.addData("Current time",runtime.seconds());
            opMode.telemetry.update();
            opMode.waitOneFullHardwareCycle();
        }

        robot.getBLM().setPower(0.0);
        robot.getBRM().setPower(0.0);
    }

    public boolean isOnLine() throws InterruptedException{
        return ods.getLightDetected() > EOPDWHITELINELIGHTLEVEL;
    }


  /*  public void readAndPush(String analysis,int goalTries, String teamOn) throws InterruptedException{ //Might not work if blue is on right...
        //Assuming the beacon is randomized, which it should be.....
        opMode.telemetry.addData("AutoStatus","Attempting to read and push");
        opMode.telemetry.update();
        int tries = 0;
        opMode.waitOneFullHardwareCycle();
        if (analysis.equals("redBlue")) { //Blue is on left
            if (teamOn.toLowerCase().equals("blue")) {
                servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
            } else {
                servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            }
            encoderDrive(.4, 100, 100, 5);
            encoderDrive(.4, -100, -100, 5);
            if (teamOn.toLowerCase().equals("blue")) {
                servoControllerLib.setDegrees(0);
            } else {
                servoControllerLib.setDegrees(180);
            }
            opMode.telemetry.addData("read and push status","Pushed left");
            opMode.telemetry.update();
        } else {
            if (teamOn.toLowerCase().equals("red")) {
                servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
            } else {
                servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            }
            encoderDrive(.4, 100, 100, 5);
            encoderDrive(.4, -100, -100, 5);
            if (teamOn.toLowerCase().equals("red")) {
                servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
            } else {
                servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            }
            opMode.telemetry.addData("read and push status","Pushed right");
            opMode.telemetry.update();
        }
        String ourAnalysis = ftcVisionManager.readBeacon(9,10);
        if (ourAnalysis.equals(analysis)){ //Add && !(analysis.equals("blueRed")) if it doesnt work with Blue on the right
            opMode.telemetry.addData("read and push status","Trying again in 5 seconds");
            opMode.telemetry.update();
            if (tries < goalTries) {
                delay(5000); //Waits 5 seconds
                tries++;
                readAndPush(ourAnalysis,goalTries,teamOn);
            } else {
                opMode.telemetry.addData("read and push status","Ran out of tries :(");
                opMode.telemetry.update();
            }
        }
        opMode.telemetry.addData("read and push status","Successfully completed! Tries: " + tries);
        opMode.telemetry.update();
    }*/

    public void readAndSetServo(String teamOn) throws InterruptedException{
        opMode.telemetry.addData("AutoStatus","Attempting to read and push");
        opMode.telemetry.update();
        int tries = 0;
        opMode.waitOneFullHardwareCycle();
        String colors = ftcVisionManager.readBeacon(9,10);
        if (colors.equals("redBlue")) {
            if(teamOn.toLowerCase() == "blue"){
                servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
            }
            else{
                servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            }
        }
        else{
            if(teamOn.toLowerCase() == "red"){
                servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
            }
            else{
                servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            }


        }

        opMode.telemetry.addData("read and set servo status","Successfully completed!");
        opMode.telemetry.update();

    }



    public void resetAll() throws InterruptedException{
        this.resetEncoders();
        setRUNWITHENCODERS();
        runtime.reset();
    }

    final boolean DEBUG = true;
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
