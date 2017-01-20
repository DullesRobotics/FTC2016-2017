package com.dullesrobotics.ftc.libraries;

import com.dullesrobotics.ftc.mods.SensorListener;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import static com.dullesrobotics.ftc.libraries.commonMethods.delay;

/**
 * Created by Kenneth on 1/8/2017.
 */

public class AutonomousDriveClass {
    BasicRobot robot;
    final static double ENCODERTICKSPERREVOLUTION = 1478.4;
    final static double CIRCUMFERENCEOFWHEELCENTIMETERS = Math.PI*9.6;
    final static double TICKSPERCENTIMETER = ENCODERTICKSPERREVOLUTION/CIRCUMFERENCEOFWHEELCENTIMETERS;
    final static double DISTANCEBETWEENWHEELSINCHES = 14.0;
    final static double POINTTURNRADIUSCM = DISTANCEBETWEENWHEELSINCHES/2.0*2.54;
    final static double SWINGTURNRADIUSCM = DISTANCEBETWEENWHEELSINCHES * 2.54;
    public final static double EOPDWHITELINELIGHTLEVEL = 0.15;
    private boolean isReversed = false;
    OpticalDistanceSensor ods;
    LinearVisionOpMode opMode;
    private ElapsedTime runtime;
    //SensorListener sensorListener;
    //float heading;
/*
    public AutonomousDrive(BasicRobot r, SensorListener s) {
        robot = r;
        sensorListener = s;
    }
*/
    public AutonomousDriveClass(BasicRobot r) {
        robot = r;
    }
    public AutonomousDriveClass(LinearVisionOpMode op, BasicRobot r, OpticalDistanceSensor o) {
        robot = r;
        ods = o;
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        opMode = op;
        runtime  = new ElapsedTime();
    }

    public AutonomousDriveClass(BasicRobot r,OpticalDistanceSensor o){
        robot = r;
        ods = o;
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        runtime  = new ElapsedTime();
    }

    public int getMaxCurrentPos(){
        return Math.max(robot.getBLM().getCurrentPosition(),robot.getBRM().getCurrentPosition());
    }

    public void setRUNWITHENCODERS(){
        robot.getBLM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setRUNWITHOUTENCODERS(){
        robot.getBLM().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setRUNTOPOSITION(){
        robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetEncoders(){
        robot.getBLM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        robot.getBLM().setZeroPowerBehavior(zeroPowerBehavior);
        robot.getBRM().setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void switchDirection(){
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
    public void setDirectionReverse(){
        robot.getBLM().setDirection(DcMotorSimple.Direction.FORWARD);
        robot.getBRM().setDirection(DcMotorSimple.Direction.REVERSE);
        isReversed = true;
    }
    public void setDirectionNotReversed(){
        //Unreverse
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        robot.getBRM().setDirection(DcMotorSimple.Direction.FORWARD);
        isReversed = false;
    }
    public void driveTillLine(double power,double timeoutS,double EOPDTriggerLevel) throws InterruptedException{
        //Go straight till EOPD
        setRUNWITHENCODERS();
        robot.getBLM().setPower(power);
        robot.getBRM().setPower(power);
        while(opMode.opModeIsActive()&& ods.getLightDetected() < EOPDTriggerLevel){opMode.waitOneFullHardwareCycle();}
        robot.getBLM().setPower(0.0);
        robot.getBRM().setPower(0.0);
    }

    public void pointTurn(double power,double deg,double timeoutS) throws InterruptedException {
        //delay(3000);
        if(deg == 0.0){
            return;
        }
        double leftDistCM = 2.0 * Math.PI * POINTTURNRADIUSCM * deg / 360.0;
        double rightDistCM = -leftDistCM;
        encoderDrive(power,leftDistCM,rightDistCM,timeoutS);
    }

    public void swingTurn(double power,double deg,double timeoutS) throws InterruptedException {
        //opMode.telemetry.addData("Old deg vs New deg",deg + " vs " + deg * 20);
        //opMode.telemetry.update();
        deg *= 200;
        if(deg > 0){
            double leftDistCM = 2.0 * Math.PI * SWINGTURNRADIUSCM * deg / 360.0;
            double rightDistCM = 0.0;
            encoderDrive(power,leftDistCM,rightDistCM,timeoutS);
        }else if (deg < 0){
            deg = Math.abs(deg);
            double leftDistCM = 0.0;
            double rightDistCM = 2.0 * Math.PI * SWINGTURNRADIUSCM * deg / 360.0;
            encoderDrive(power,leftDistCM,rightDistCM,timeoutS);
        }
    }

    public boolean encoderDrive  (double speed,
                             double leftCM, double rightCM,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;
        //robot.getBLM().set
        this.resetEncoders();
        boolean hitTimeOut = false;
        // Ensure that the opmode is still active
        //delay(250);
        leftCM *= 5;
        rightCM *= 5;
        if (opMode.opModeIsActive()) {
            debug(1);
            //delay(250);
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.getBLM().getCurrentPosition() + (int)(leftCM * TICKSPERCENTIMETER);
            newRightTarget = robot.getBRM().getCurrentPosition() + (int)(rightCM * TICKSPERCENTIMETER);
            robot.getBLM().setTargetPosition(newLeftTarget);
            robot.getBRM().setTargetPosition(newRightTarget);
            debug(2);
            delay(250);
            // Turn On RUN_TO_POSITION
            robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            debug(3);
            delay(250);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.getBLM().setPower(Math.abs(speed));
            robot.getBRM().setPower(Math.abs(speed));
            debug(4);
            delay(250);
            // keep looping while we are still active, and there is time left, and both motors are running.
            opMode.telemetry.addData("Status1",opMode.opModeIsActive());
            opMode.telemetry.addData("Status2",(runtime.seconds() < timeoutS));
            opMode.telemetry.addData("Status3",(robot.getBLM().isBusy() && robot.getBRM().isBusy()));
            opMode.telemetry.update();
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.getBLM().isBusy() && robot.getBRM().isBusy())) {
                opMode.waitOneFullHardwareCycle();

                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                opMode.telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.getBLM().getCurrentPosition(),
                        robot.getBRM().getCurrentPosition());
                opMode.telemetry.update();
                debug(5);
                if(runtime.seconds() > timeoutS){
                    hitTimeOut = true;
                    break;
                }
                debug(6);
               // if (!opMode.opModeIsActive()){
                   // opMode.stop();
                    //break;
                //}
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
        return hitTimeOut;
    }
    public boolean isReversed(){
        return isReversed;
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

    public void turnTillLine(double power, double EOPDThreshold, boolean turnLeft) throws InterruptedException {
        setRUNWITHENCODERS();
        if(turnLeft){
            robot.getBRM().setPower(power);
            while(ods.getLightDetected() < EOPDThreshold&&opMode.opModeIsActive()){ opMode.waitOneFullHardwareCycle();}
            robot.getBRM().setPower(0.0);
        }else{
            robot.getBLM().setPower(power);
            while(ods.getLightDetected() < EOPDThreshold&&opMode.opModeIsActive()){opMode.waitOneFullHardwareCycle();}
            robot.getBLM().setPower(0.0);
        }
    }
    public void turnRightTillOffLine(double power, double EOPDThreshold) throws InterruptedException {
        robot.getBLM().setPower(power);
        while(opMode.opModeIsActive()&&ods.getLightDetected() > EOPDThreshold){opMode.waitOneFullHardwareCycle();}
        robot.getBLM().setPower(0.0);
    }
    public void followLine(double power,double EOPDThreshold,double timeoutS,boolean leftFirst) throws InterruptedException {
        turnTillLine(power,EOPDThreshold,leftFirst);
        runtime.reset();
        while(opMode.opModeIsActive()&&runtime.seconds()<timeoutS){
            turnRightTillOffLine(power,EOPDThreshold);
            turnTillLine(power,EOPDThreshold,true);
            opMode.waitOneFullHardwareCycle();
        }
        robot.getBLM().setPower(0.0);
        robot.getBRM().setPower(0.0);
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
