package org.firstinspires.ftc.teamcode.skystone.TB0;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public abstract class BaseClassTB0 extends LinearOpMode {

    //Final variables (in inches)
    final static double wheelDiameter = 4.0;
    final static double wheelDistance = 14.375;
    //Variables for tuning position
    static int valMid = -1;
    static int valLeft = -1;
    static int valRight = -1;
    static float rectHeight = .6f / 8f;
    static float rectWidth = 1.5f / 8f;
    static float offsetX = 0f / 8f;
    static float offsetY = 0f / 8f;
    static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};
    static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
    static String skystonePos = "unknown";
    final int rows = 640;
    final int cols = 480;
    DcMotor mBL;//Back left
    DcMotor mBR;
    DcMotor mFL;
    DcMotor mFR;//Front right
    DcMotor mIL;
    DcMotor mIR;
    //DcMotor mML;
    //DcMotor mMR;
    DcMotor mA;
    //Servo sP;
    Servo sC;
    BNO055IMU imu; //REV gyro
    Orientation angles;
    ElapsedTime runtime;
    ElapsedTime stopTime;
    OpenCvCamera phoneCam;
    DistanceSensor d1; //Front
    ColorSensor c1;
    Orientation lastAngle = new Orientation();
    //double enXprediction = 0;
    Pose pose = new Pose(0, 0, 0);
    //Global sensor values
    float gyroZ;
    int currOdY1;
    int currOdY2;
    int prevOdY1 = 0;
    int prevOdY2 = 0;
    int deltaOdY1 = 0;
    int deltaOdY2 = 0;
    double currTheta = 0;
    double deltaTheta = 0;
    double arcLength = 0;
    int prevEnX = 0;
    int prevEnYLeft = 0;
    int prevEnYRight = 0;
    double prevGyro = 0;
    double posX = 0;
    double posY = 0;

    public static double ticksToInches(int ticks) {
        double circum = Math.PI * wheelDiameter;
        return (circum / 1400) * ticks;
    }

    public static double[] arcInfo(int deltaLeft, int deltaRight) {
        double inchesLeft = ticksToInches(deltaLeft);
        double inchesRight = ticksToInches(deltaRight);
        double deltaTheta = (inchesRight - inchesLeft) / wheelDistance;
        double arcLength = (inchesRight + inchesLeft) / 2;
        return new double[]{deltaTheta, arcLength};
    }

    //Defines all components for init()
    public void defineComponents() {

        mBL = hardwareMap.dcMotor.get("mBL");//Back left
        mBR = hardwareMap.dcMotor.get("mBR");
        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");//Front right
        mIL = hardwareMap.dcMotor.get("mIL");
        mIR = hardwareMap.dcMotor.get("mIR");
        //mML = hardwareMap.dcMotor.get("mML");
        //mMR = hardwareMap.dcMotor.get("mMR");
        mA = hardwareMap.dcMotor.get("mA");
        //sP = hardwareMap.servo.get("sP");
        sC = hardwareMap.servo.get("sC");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//Open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//Display image on robot controller

        // mEnX = hardwareMap.dcMotor.get("mEnX");
        // mEnY = hardwareMap.dcMotor.get("mEnY");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //Calibration file
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        d1 = hardwareMap.get(DistanceSensor.class, "d1");
        c1 = hardwareMap.get(ColorSensor.class, "c1");

        mBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set motors on right side of robot backwards
        mBR.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.REVERSE);
        mIL.setDirection(DcMotor.Direction.REVERSE);
        //mMR.setDirection(DcMotor.Direction.REVERSE);

        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime = new ElapsedTime();
        stopTime = new ElapsedTime();
    }

    public Pose getPose() {
        return this.pose;
    }

    public void updatePose() {
        currOdY1 = mBL.getCurrentPosition();
        currOdY2 = -mFL.getCurrentPosition();

        deltaOdY1 = currOdY1 - prevOdY1;
        deltaOdY2 = currOdY2 - prevOdY2;

        deltaTheta = arcInfo(deltaOdY1, deltaOdY2)[0];
        arcLength = arcInfo(deltaOdY1, deltaOdY2)[1];

        posX += arcLength * Math.cos(currTheta + (deltaTheta / 2));
        posY += arcLength * Math.sin(currTheta + (deltaTheta / 2));

        prevOdY1 = currOdY1;
        prevOdY2 = currOdY2;
        currTheta += deltaTheta;

        pose.x = posX;
        pose.y = posY;
        pose.theta = currTheta;
    }

    public void updatePoseStrafe() {

        //Set current values for iteration
        int currEnX = mFR.getCurrentPosition();
        int currEnYLeft = mBL.getCurrentPosition();
        int currEnYRight = -mFL.getCurrentPosition();
        double currGyro = -gyroZ;

        int changeEnYLeft = currEnYLeft - prevEnYLeft;
        int changeEnYRight = currEnYRight - prevEnYRight;
        double changeGyro = currGyro - prevGyro;

        //Get change in encoder values
        double xRotateGuess = (changeEnYLeft - changeEnYRight) / 1.57676254; //Constant = y ticks / x ticks for given rotation 1.57676254, 1.611357068191601
        double changeEnXVirtual = currEnX - prevEnX + xRotateGuess;
        //enXprediction += currEnX - prevEnX + xRotateGuess; //Only used to compare to 2nd enX

        double changeEnXPhysical = currEnX - prevEnX;
        double changeEnX = (changeEnXPhysical + changeEnXVirtual) / 2;
        double changeEnY = (changeEnYLeft + changeEnYRight) / 2;

        //Update odometer values
        posX += (Math.cos(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnY) - (Math.sin(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnX);
        posY -= (Math.sin(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnY) + (Math.cos(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnX);

        //Set pose variables in inches
        pose.x = ticksToInches((int) posX);
        pose.y = ticksToInches((int) posY);
        pose.theta = Math.toRadians(gyroZ);

        //Set previous values for next iteration
        prevEnX = currEnX;
        prevEnYLeft = currEnYLeft;
        prevEnYRight = currEnYRight;
        prevGyro = currGyro;
    }



    public boolean isInTolerance(double targetX, double targetY, double tolerance) {
        if (Math.abs(pose.x - targetX) < tolerance && Math.abs(pose.y - targetY) < tolerance) {
            return true;
        } else {
            return false;
        }
    }

    //Moves drive train in all possible directions
    public void drive(double forward, double strafe, double rotate) {
        mFL.setPower(forward + strafe + rotate);
        mFR.setPower(forward - strafe - rotate);
        mBL.setPower(forward - strafe + rotate);
        mBR.setPower(forward + strafe - rotate);
    }

    public void driveBackward(double power) {
        driveForward(-power);
    }

    public void driveForward(double power) {
        drive(power, 0, 0);
        return;
    }

    public void driveLeft(double power) {
        drive(0, -power, 0);
    }

    public void driveRight(double power) {
        driveLeft(-power);
    }

    public void gyroUpdate() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float originalAngle = angles.firstAngle - lastAngle.firstAngle;

        if (originalAngle < -180) {
            originalAngle += 360;
        } else if (originalAngle > 180) {
            originalAngle -= 360;
        }

        lastAngle = angles;
        gyroZ += originalAngle;

    }

    //Inverse sigmoid function for use in drivetrain directional movement
    public double inverseSigmoid(float input) {
        double value = Math.log((1 - input) / (input + 1)) / -5;
            if (value > 1) {
            value = 1;
        } else if (value < -1) {
            value = -1;
        } else {
            return value;
        }
        return value;
    }

    public void setIntakePower(double power) {
        mIL.setPower(power);
        mIR.setPower(power);
    }


    public void odometerUpdate() {

        float gyroEuclid = gyroZ % 360;

        if (gyroEuclid < 0) {
            gyroEuclid += 360;
        }
/*
        //Set current values for iteration
        currEnX = mFR.getCurrentPosition();
        currEnYLeft = -mFL.getCurrentPosition();

        //Get change in encoder values
        changeEnX = currEnX - prevEnX;
        changeEnYLeft = currEnYLeft - prevEnYLeft;
        changeDeg = gyroZ - prevDeg;

        //Adjust for rotation (measured constants are in ticks/degree)
        changeEnX = (int)(changeEnX - (changeDeg * -12.8877098));
        changeEnYLeft = (int)(changeEnYLeft - (changeDeg * -15.6436288));

        //Calculate multiplier for percent of encoder resulting in odometer values
        multiplier = ((2 - Math.sqrt(2)) / 4) * Math.cos((Math.PI / 180) * 4 * gyroEuclid) + ((2 + Math.sqrt(2)) / 4);

        //Adjust for quadrant
        if(gyroEuclid >= 45 && gyroEuclid < 135){
            changeOdX = multiplier * changeEnYLeft;
            changeOdY = multiplier * -changeEnX;
        } else if(gyroEuclid >= 135 && gyroEuclid < 225){
            changeOdX = multiplier * -changeEnX;
            changeOdY = multiplier * -changeEnYLeft;
        } else if(gyroEuclid >= 225 && gyroEuclid < 315){
            changeOdX = multiplier * -changeEnYLeft;
            changeOdY = multiplier * changeEnX;
        } else {
            changeOdX = multiplier * changeEnX;
            changeOdY = multiplier * changeEnYLeft;
        }


        //Update odometer values
        odX += changeOdX;
        odY += changeOdY;

        //Set previous values for next iteration
        prevEnX = currEnX;
        prevEnYLeft = currEnYLeft;
        prevDeg = gyroZ; */
    }

    public void rotateClockwise(double power) {
        drive(0, 0, power);
    }

    public void rotateCounterclockwise(double power) {
        rotateClockwise(-power);
    }

    public void stopDriveTrain() {
        drive(0, 0, 0);
    }

    public void moveToPoseTele(int targetX, int targetY, int forward, boolean drive, boolean rotateBeforeDrive) {

        double distanceX = (targetX - pose.x);
        double distanceY = (targetY - pose.y);
        double altSpeed;

        double tangentOf = (targetY - pose.y) / distanceX;
        double distanceToTarget = Math.sqrt(Math.pow((distanceX), 2) + Math.pow(distanceY, 2));

        double distanceToTheta = (Math.atan(tangentOf) - pose.theta);

        if (distanceToTheta > Math.PI / 2) {
            distanceToTheta -= Math.PI;
        }

        if (distanceToTheta < -Math.PI / 2) {
            distanceToTheta += Math.PI;
        }

        double distanceSpeed = (2 / (1 + Math.pow(Math.E, -0.03 * distanceToTarget))) - 1;

        if (Math.abs(Math.toDegrees(distanceToTheta)) > 30 && rotateBeforeDrive) {
            distanceSpeed = 0;
        }

        double leftSidePower = 0;
        double rightSidePower = 0;

        if (distanceToTheta < 0) {
            if (forward > 0) {
                leftSidePower = distanceSpeed * forward; // + orientationSpeed;
                rightSidePower = distanceSpeed * (Math.cos(distanceToTheta) / 6) * forward; // - orientationSpeed;
                //telemetry.addData("Left Side Dominates", "");
            } else {
                leftSidePower = distanceSpeed * (Math.cos(distanceToTheta) / 6) * forward; // - orientationSpeed;
                rightSidePower = distanceSpeed * forward; // + orientationSpeed;
                //telemetry.addData("Right Side Dominates", "");
            }
        } else {
            if (forward > 0) {
                leftSidePower = distanceSpeed * (Math.cos(distanceToTheta) / 6) * forward; // - orientationSpeed;
                rightSidePower = distanceSpeed * forward; // + orientationSpeed;
                //telemetry.addData("Right Side Dominates", "");
            } else {
                leftSidePower = distanceSpeed * forward; // + orientationSpeed;
                rightSidePower = distanceSpeed * (Math.cos(distanceToTheta) / 6) * forward; // - orientationSpeed;
                //telemetry.addData("Left Side Dominates", "");
            }
        }

        if (drive) {
            mFL.setPower(leftSidePower);
            mFR.setPower(rightSidePower);
            mBL.setPower(leftSidePower);
            mBR.setPower(rightSidePower);
        }
        /*
        telemetry.addData("Pose X", pose.x);
        telemetry.addData("Pose Y", pose.y);
        telemetry.addData("Distance to Theta", Math.toDegrees(distanceToTheta));
        telemetry.addData("Tan", Math.toDegrees(Math.atan(tangentOf)));
        telemetry.addData("Left Side Power", leftSidePower);
        telemetry.addData("Right Side Power", rightSidePower);
        telemetry.update();
        */
    }
    /*
    public String skystonePosition() {
        if (valLeft < valRight && valLeft < valMid) {
            return "left";
            //Return left
        } else if (valMid < valRight && valMid < valLeft) {
            return "middle";
            //Return middle
        } else if (valRight < valLeft && valRight < valMid) {
            return "right";
            //Return right
        } else {
            return "unknown";
            //Return none
        }
    }
    */

    //Detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline {
        //Defines matrices
        Mat yCrCbChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        private DetectSkystones.StageSwitchingPipeline.Stage stageToRenderToViewport = DetectSkystones.StageSwitchingPipeline.Stage.RAW_IMAGE;
        private DetectSkystones.StageSwitchingPipeline.Stage[] stages = DetectSkystones.StageSwitchingPipeline.Stage.values();

        @Override
        public Mat processFrame(Mat input) {

            //Default all white
            int leftColor = 255;
            int midColor = 255;
            int rightColor = 255;

            //Make the skystone box purple
            if (valLeft < valRight && valLeft < valMid) {
                leftColor = 0;
                midColor = 255;
                rightColor = 255;
                //Return left
            } else if (valMid < valRight && valMid < valLeft) {
                leftColor = 255;
                midColor = 0;
                rightColor = 255;
                //Return middle
            } else if (valRight < valLeft && valRight < valMid) {
                leftColor = 255;
                midColor = 255;
                rightColor = 0;
                //Return right
            }

            //Find contours of yellow objects
            contoursList.clear();
            Mat shapesMat = input;

            Imgproc.cvtColor(input, yCrCbChan2Mat, Imgproc.COLOR_RGB2YCrCb);//Converts rgb input to yCrCb
            Core.extractChannel(yCrCbChan2Mat, yCrCbChan2Mat, 2);//Extracts Cb (blue yellow) channel to threshold
            Imgproc.threshold(yCrCbChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);//Threshold at a Cb value of 102

            //Resets sum values to 0 before each iteration
            valMid = 0;
            valLeft = 0;
            valRight = 0;

            //Sums values of an 11 pixel square from midpoint
            for (int pixoffsetX = -2; pixoffsetX <= 2; pixoffsetX++) {
                for (int pixoffsetY = -2; pixoffsetY <= 2; pixoffsetY++) {
                    double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]) + pixoffsetY, (int) (input.cols() * midPos[0]) + pixoffsetX);
                    valMid += (int) pixMid[0];
                    double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]) + pixoffsetY, (int) (input.cols() * leftPos[0]) + pixoffsetX);
                    valLeft += (int) pixLeft[0];
                    double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]) + pixoffsetY, (int) (input.cols() * rightPos[0]) + pixoffsetX);
                    valRight += (int) pixRight[0];
                }
            }

            //Draw squares to represent pixel sums
            Imgproc.rectangle(
                    shapesMat,
                    new Point((int) (input.cols() * leftPos[0]) - 5, (int) (input.rows() * leftPos[1]) - 5),
                    new Point((int) (input.cols() * leftPos[0]) + 5, (int) (input.rows() * leftPos[1]) + 5),
                    new Scalar(255, leftColor, 255), 1);
            Imgproc.rectangle(
                    shapesMat,
                    new Point((int) (input.cols() * midPos[0]) - 5, (int) (input.rows() * midPos[1]) - 5),
                    new Point((int) (input.cols() * midPos[0]) + 5, (int) (input.rows() * midPos[1]) + 5),
                    new Scalar(255, midColor, 255), 1);
            Imgproc.rectangle(
                    shapesMat,
                    new Point((int) (input.cols() * rightPos[0]) - 5, (int) (input.rows() * rightPos[1]) - 5),
                    new Point((int) (input.cols() * rightPos[0]) + 5, (int) (input.rows() * rightPos[1]) + 5),
                    new Scalar(255, rightColor, 255), 1);

            //Draw rectangles for boundary of stones
            Imgproc.rectangle(
                    shapesMat,
                    new Point(input.cols() * (leftPos[0] - rectWidth / 2), input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(input.cols() * (leftPos[0] + rectWidth / 2), input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(255, leftColor, 255), 2);
            Imgproc.rectangle(
                    shapesMat,
                    new Point(input.cols() * (midPos[0] - rectWidth / 2), input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(input.cols() * (midPos[0] + rectWidth / 2), input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(255, midColor, 255), 2);
            Imgproc.rectangle(
                    shapesMat,
                    new Point(input.cols() * (rightPos[0] - rectWidth / 2), input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(input.cols() * (rightPos[0] + rectWidth / 2), input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(255, rightColor, 255), 2);

            if (valLeft < valRight && valLeft < valMid) {
                skystonePos = "left";
                //Return left
            } else if (valMid < valRight && valMid < valLeft) {
                skystonePos = "middle";
                //Return middle
            } else if (valRight < valLeft && valRight < valMid) {
                skystonePos = "right";
                //Return right
            } else {
                skystonePos = "unknown";
                //Return none
            }

            //Cases for tap pipeline
            switch (stageToRenderToViewport) {

                case THRESHOLD: {
                    return thresholdMat;
                }
                default: {
                    return input;
                }
            }

        }

        //Stages for tap pipeline
        enum Stage {
            THRESHOLD,
            RAW_IMAGE,
        }
    }
}