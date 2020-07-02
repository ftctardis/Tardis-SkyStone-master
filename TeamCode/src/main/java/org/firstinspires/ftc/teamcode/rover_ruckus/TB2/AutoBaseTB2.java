package org.firstinspires.ftc.teamcode.rover_ruckus.TB2;

//Imports
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public abstract class AutoBaseTB2 extends BaseClassTB2 {

    //Global variables
    int goldPos = 0;
    int mineralsInView = 0;
    int armSamplePos;
    double encoderAvg = 0;
    double lastEncoder = 0;
    double dFCMPrev = 0;
    double targetCLEAR_LANDER;
    double targetDRIVE_TO_WALL;
    double targetSTRAFE_TO_GOLD;
    double targetPREPARE_TO_CLAIM;
    double targetFACE_MINERAL;
    double targetSAMPLE;
    double targetSAMPLE_Silver;
    double targetDRIVE_TO_CRATER;
    double lowerSSTime;
    double stopTime;
    boolean isRobotDetected = true;

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public static final String VUFORIA_KEY = "Ad/RkS3/////AAABmT3n7HzeHUPEmBy6VKmkTD9iZTFdcI0bOnzexuBXujebew6SLb+BmyefGULiC+mXd3kjAzeq9mGm5ih9bIW3EYarjDjYtl0T/QJDsCZpo68+k7NAupYTif3USRbwqi2ioyw0KsOIEOkMuNzZ36KAV9B2TMZ5UcHlKHxXf17ZdEYiwwc8OEVu+VyrCbtxEBIt5WmEAqtlUEI79QUjAETWPpewYKW6pN3lPMcxPLJuEiIxb+0hnavvM3CEeXq1c6+yA+EKizK9DXr9g8UrpwbS9mSScWSI9sHMgImRstM1H5IXWmk0ZtWKRMHKFPvrC752JXS2oIKLA141gUm8wMZAAhyF6IXxGcUtuBqx+FURtE5y.";


    //Cases for auto steps
    public enum steps {

        //40, 45
        CLEAR_ARM,
        DROP_LANDER,
        DRIVE_TO_DEPOT, PARK,
        DROP_ARM,
        DROP_MARKER,
        EXIT_DEPOT,
        TRY_FOR_SAMPLING,
        STOP,

        //65, 70
        SCAN_MINERALS,
        //DROP_LANDER
        CLEAR_LANDER,
        FACE_WALL,
        DRIVE_TO_WALL,
        STRAFE_TO_GOLD,
        FACE_DEPOT, FACE_CRATER,
        PREPARE_TO_CLAIM,
        ROTATE_CLAIM,
        WALL_ALIGN,
        FAST_CLAIM, EXTENDED_CLAIM,
        //STOP

        //80
        //SCAN_MINERALS
        FACE_MINERAL,
        SAMPLE,
        DRIVE_TO_CRATER,
        DRIVE_TO_CRATER_BACKWARDS,
        ADJUST_DEPOT,

        LEFT_FACE_DEPOT,
        LEFT_PREPARE_TO_CLAIM,
        LEFT_FACE_CRATER,

        CENTER_PREPARE_TO_CLAIM,
        CENTER_EXIT_DEPOT,
        CENTER_FACE_CRATER,

        RIGHT_FACE_DEPOT,
        RIGHT_EXIT_DEPOT,
        RIGHT_PREPARE_TO_CLAIM,
        RIGHT_PREPARE_TO_CLAIM2,
        AVOID_MINERAL,
        RIGHT_FACE_CRATER,
        POSITION_TO_FACE_WALL

    }


    //Activates the Tensor Flow Object Detection engine
    public void activateTfod() {
        if (tfod != null) {
            tfod.activate();
        } else {
            return;
        }
    }



    //Drives forward using a sigmoid to start fast and slow down while close to target
    public void driveDistanceSigmoid(double target, double degree, double tolerance) {
        double power = (1 / (1 + Math.pow(Math.E, -(0.07 * (dF.getDistance(DistanceUnit.CM) - target))))) - 0.5;
        if (power < 0.08) {
            power = 0.08;
        }
        driveForwardGyro(power, degree, tolerance);
    }


    //Drives forward while correcting to face designated gyro heading
    public void driveForwardGyro(double power, double degree, double tolerance) {
        if (gyroZ + tolerance < degree) {
            mFL.setPower(power);
            mFR.setPower(power + 0.1);
            mBL.setPower(power);
            mBR.setPower(power + 0.1);
        } else if (gyroZ - tolerance > degree) {
            mFL.setPower(power + 0.1);
            mFR.setPower(power);
            mBL.setPower(power + 0.1);
            mBR.setPower(power);
        } else {
            driveForward(power);
        }
    }


    //Drives right while correcting to face designated gyro heading
    public void driveRightGyro(double power, double degree, double tolerance) {
        if (gyroZ + tolerance < degree) {
            mFL.setPower(power);
            mFR.setPower(-power + 0.1);
            mBL.setPower(-power);
            mBR.setPower(power + 0.1);
        } else if (gyroZ - tolerance > degree) {
            mFL.setPower(power + 0.1);
            mFR.setPower(-power);
            mBL.setPower(-power + 0.1);
            mBR.setPower(power);
        } else {
            driveRight(power);
        }
    }


    //Takes average of encoders on mFR and mBL
    public void getEncoderAvg() {
        encoderAvg = ((mFR.getCurrentPosition() + mBL.getCurrentPosition()) / 2) - lastEncoder;
    }


    //Image recognition logic for finding relative gold position for sampling
    public int getGoldPos() throws NullPointerException {

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                mineralsInView = updatedRecognitions.size();

                //Remove all but the two lowest
                while (updatedRecognitions.size() > 2) {
                    float highestRecognition = 1000000000;
                    int idxHighest = -1;
                    for (int i = 0; i < updatedRecognitions.size(); i++) {
                        Recognition r = updatedRecognitions.get(i);
                        if (r.getBottom() < highestRecognition) {
                            idxHighest = i;
                            highestRecognition = r.getBottom();
                        }
                    }
                    updatedRecognitions.remove(idxHighest);
                }

                if (updatedRecognitions.size() == 2) {

                    int mineralOne = -1;
                    int mineralTwo = -1;
                    boolean isMineralOneGold = false;
                    boolean isMineralTwoGold = false;

                    for (Recognition rec : updatedRecognitions) {
                        if (mineralOne == -1) {
                            //First mineral
                            mineralOne = (int) rec.getLeft();
                            isMineralOneGold = (rec.getLabel().equals(LABEL_GOLD_MINERAL));
                        } else {
                            //Second Mineral
                            mineralTwo = (int) rec.getLeft();
                            isMineralTwoGold = (rec.getLabel().equals(LABEL_GOLD_MINERAL));
                        }
                    }
                    //No gold Recognized
                    if (!isMineralOneGold && !isMineralTwoGold) {
                        goldPos = 3;
                    }
                    //Gold was recognized
                    else {
                        if (isMineralOneGold) { //First was Gold
                            goldPos = (mineralOne < mineralTwo) ? 1 : 2; //Ternary operator
                        } else { //Second was Gold
                            goldPos = (mineralTwo < mineralOne) ? 1 : 2;
                        }
                    }
                }
            }
        } else {
            throw new NullPointerException("Tfod not active");
        }
        telemetry.addData("Recognitions: ", mineralsInView);
        telemetry.addData("Gold position: ", goldPos);
        telemetry.update();
        return goldPos;
    }


    public void gyroAdjust(double power, double degree) {

        if (gyroZ > degree) {
            rotateClockwise(power);
        } else if (gyroZ < degree) {
            rotateCounterclockwise(power);
        } else {
            return;
        }
    }


    //Initialize the Tensor Flow Object Detection engine
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    //Initialize the Vuforia localization engine
    public void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = CameraDirection.BACK; //Phone camera
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1"); //Webcam camera
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }


    public void lowerSSAuto() {
        if (matchTime.seconds() > lowerSSTime && matchTime.seconds() < lowerSSTime + 2) {
            mSS.setPower(-1);
        } else {
            mSS.setPower(0);
        }
    }


    //Resets encoderAvg and runtime variables to 0
    public void resetSteps() {
        lastEncoder = (mFR.getCurrentPosition() + mBL.getCurrentPosition()) / 2;
        runtime.reset();
        stopDriveTrain();
    }


    //Sigmoid function for rotating clockwise
    public void rotateSigmoid(double degree) {
        double power = (1 / (1 + Math.pow(Math.E, -(0.05 * (gyroZ - degree))))) - 0.5;
        if (power < 0.07 && power > 0) {
            power = 0.07;
        }
        if (power > -0.07 && power < 0) {
            power = -0.07;
        }
        rotateClockwise(power);
    }


    //Displays all sensor readings and important variables for debugging
    public void sensorTelemetry() {
        gyroUpdate();

        double dFCM = dF.getDistance(DistanceUnit.CM);
        double dRCM = dR.getDistance(DistanceUnit.CM);
        double mATicks = mA.getCurrentPosition();
        boolean ssLimit = !mlsSS.getState();

        telemetry.addData("Gold position: ", goldPos);
        telemetry.addData("dFCM: ", String.format("%.01f cm", dFCM));
        telemetry.addData("dRCM: ", String.format("%.01f cm", dRCM));
        telemetry.addData("Gyro: ", gyroZ);
        telemetry.addData("mA: ", String.format("%.01f ticks", mATicks));
        telemetry.addData("Encoder avg: ", encoderAvg);
        telemetry.addData("SS Limit: ", ssLimit);
        telemetry.addData("Runtime: ", String.format("%.01f sec", runtime.seconds()));
        telemetry.addData("Stop time: ", String.format("%.01f sec", stopTime));
        telemetry.update();
    }


    //Shutdown all processes and stop drivetrain
    public void shutdown() {
        stopDriveTrain();
        stop();
        shutdownTfod();
        verticalSweeper(0);
    }


    //Closes the Tensor Flow Object Detection engine
    public void shutdownTfod() {
        if (tfod != null) {
            tfod.shutdown();
        } else {
            return;
        }
    }


    //Defines values for different minerals
    public void updateMineralTarget() {

        armSamplePos = -200;
        targetDRIVE_TO_CRATER = 300;

        if (goldPos == 1 || goldPos == 0) {
            //65, 70
            //targetCLEAR_LANDER = 400;
            //targetDRIVE_TO_WALL = 60;
            //targetSTRAFE_TO_GOLD = -200;
            //targetPREPARE_TO_CLAIM = 0;

            //80
            targetFACE_MINERAL = 30;
            targetSAMPLE = 1200;
            targetSAMPLE_Silver = 1200;
            targetDRIVE_TO_WALL = 1300;


        } else if (goldPos == 2) {
            //65, 70
            //targetCLEAR_LANDER = 1200;
            //targetDRIVE_TO_WALL = 91;
            //targetSTRAFE_TO_GOLD = -300;
            //targetPREPARE_TO_CLAIM = 500;

            //80
            targetFACE_MINERAL = 0;
            targetSAMPLE = 2400;
            targetSAMPLE_Silver = 850;
            targetDRIVE_TO_WALL = 1700;

        } else {
            //65, 70
            //targetCLEAR_LANDER = 1000;
            //targetSTRAFE_TO_GOLD = -1800;
            //targetPREPARE_TO_CLAIM = 700;

            //80
            targetFACE_MINERAL = -28;
            targetSAMPLE = 1600;
            targetSAMPLE_Silver = 1200;
            targetDRIVE_TO_WALL = 2200;
        }
    }
}