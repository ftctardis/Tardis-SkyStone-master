package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;

public abstract class BaseClassTB1 extends LinearOpMode {

    //Final variables (in inches)
    final static double wheelDiameter = 4.0;
    final static double wheelDistance = 16.25;
    DcMotor mBL;//Back left
    DcMotor mBR;
    DcMotor mFL;
    DcMotor mFR;//Front right
    DcMotor mIL;//Intake left
    DcMotor mIR;
    DcMotor mML;//Mast left
    DcMotor mMR;
    Servo sP;//Park
    Servo sW;//Wrist
    //CRServo sG;
    Servo sG;//Gantry
    Servo sC;//Clamp (stone gripper)
    CRServo sIL;//Intake left
    CRServo sIR;
    Servo sFL;//Foundation left
    Servo sFR;
    Servo sCS;//Capstone
    OpenCvCamera webcam;
    BNO055IMU imu; //REV gyro
    Orientation angles;
    ElapsedTime runtime;
    ElapsedTime runtimeTwo;
    ElapsedTime stoneIntakeTime;
    //ColorSensor c1;
    ElapsedTime stopTime;
    DistanceSensor dI;//Intake stone check
    Orientation lastAngle = new Orientation();
    Pose pose = new Pose(0, 0, 0);
    //Global sensor values
    float gyroZ;
    boolean isStartRecorded = false;
    //moveToPose() global
    double thetaStart;
    double distanceToTargetStart;
    double thetaRobot;
    //Global variables
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
    //double enXprediction = 0;
    int prevEnYRight = 0;
    double prevGyro = 0;
    double posX = 0;
    double posY = 0;
    String skystonePos = "undefined";

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
        mML = hardwareMap.dcMotor.get("mML");
        mMR = hardwareMap.dcMotor.get("mMR");

        sP = hardwareMap.servo.get("sP");
        sC = hardwareMap.servo.get("sC");
        sW = hardwareMap.servo.get("sW");
        //sG = hardwareMap.crservo.get("sG");
        sG = hardwareMap.servo.get("sG");
        sIL = hardwareMap.crservo.get("sIL");
        sIR = hardwareMap.crservo.get("sIR");
        sFL = hardwareMap.servo.get("sFL");
        sFR = hardwareMap.servo.get("sFR");
        sCS = hardwareMap.servo.get("sCS");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //Calibration file
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        dI = hardwareMap.get(DistanceSensor.class, "dI");
        //c1 = hardwareMap.get(ColorSensor.class, "c1");

        mBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set motors on right side of robot backwards
        mBR.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.REVERSE);
        mIL.setDirection(DcMotor.Direction.REVERSE);
        mMR.setDirection(DcMotor.Direction.REVERSE);

        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mML.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime = new ElapsedTime();
        runtimeTwo = new ElapsedTime();
        stoneIntakeTime = new ElapsedTime();
        stopTime = new ElapsedTime();

        sW.setPosition(0.63); //.4
        sC.setPosition(0.65);
        sP.setPosition(1);
        sCS.setPosition(1);
        //sG.setPosition(0.01);
        releaseFoundation();
    }

    public void holdFoundation() {
        sFL.setPosition(0.9);
        sFR.setPosition(0.15);
    }

    public void releaseFoundation() {
        sFL.setPosition(0.5);
        sFR.setPosition(0.55);
    }

    public Pose getPose() {
        return this.pose;
    }

    public void updatePose() {
        currOdY1 = mBL.getCurrentPosition();
        currOdY2 = mFR.getCurrentPosition();

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
        int currEnX = mBR.getCurrentPosition();
        int currEnYLeft = mBL.getCurrentPosition();
        int currEnYRight = mFR.getCurrentPosition();
        double currGyro = -gyroZ;

        int changeEnYLeft = currEnYLeft - prevEnYLeft;
        int changeEnYRight = currEnYRight - prevEnYRight;
        double changeGyro = currGyro - prevGyro;

        //Get change in encoder values
        double xRotateGuess = (changeEnYLeft - changeEnYRight) * 1.025; //Constant = |avg enY| / enX (prev = 1.033)
        double changeEnXVirtual = currEnX - prevEnX + xRotateGuess;
        //enXprediction += currEnX - prevEnX + xRotateGuess; //Only used to compare to 2nd enX

        double changeEnXPhysical = currEnX - prevEnX;
        double changeEnX = (changeEnXPhysical + changeEnXVirtual) / 2;
        double changeEnY = (changeEnYLeft + changeEnYRight) / 2;

        //Update odometer values (+, -)
        posX -= (Math.cos(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnY) - (Math.sin(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnX);
        posY += (Math.sin(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnY) + (Math.cos(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnX);

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

    public void moveToPose(double targetX, double targetY, double targetTheta, double rotatePercent) {

        //Distance
        double distanceX = targetX - pose.x;
        double distanceY = targetY - pose.y;
        double distanceToTarget = Math.sqrt(Math.pow((distanceX), 2) + Math.pow(distanceY, 2));

        //Starting values
        if (!isStartRecorded) {
            distanceToTargetStart = distanceToTarget;
            thetaStart = gyroZ;
            isStartRecorded = true;
        }

        //Robot orientation vs distance to target (in degrees)
        if (distanceToTarget > distanceToTargetStart * (1 - rotatePercent)) {
            thetaRobot = (1 - (distanceToTarget - distanceToTargetStart * (1 - rotatePercent)) / (distanceToTargetStart * rotatePercent)) * (targetTheta - thetaStart) + thetaStart;
        } else {
            thetaRobot = targetTheta;
        }

        //Positive rotation = clockwise = decrease in theta
        double rotate = (1 / (1 + Math.pow(Math.E, -(0.13 * (gyroZ - thetaRobot))))) - 0.5;

        //Threshold values for motor power
        rotate = thresholdMotorPower(rotate, 0.1);

        //Powers
        double strafe = (2 / (1 + Math.pow(Math.E, -(0.08 * (distanceToTarget * Math.cos(-pose.theta - Math.atan(distanceX / distanceY) + Math.toRadians(rotate * 20))))))) - 1;
        double forward = (2 / (1 + Math.pow(Math.E, -(0.08 * (distanceToTarget * Math.sin(-pose.theta - Math.atan(distanceX / distanceY) + Math.toRadians(rotate * 20))))))) - 1;

        //Threshold values for motor power
        forward = thresholdMotorPower(forward, 0.15); //0.25
        strafe = thresholdMotorPower(strafe, 0.15);

        //Adjust for quadrants
        if (pose.y < targetY) {
            forward = -forward;
            strafe = -strafe;
        }

        drive(forward, strafe, -rotate);
    }

    public void moveToPoseWaypoint(double targetX, double targetY, double targetTheta, double rotatePercent) {

        //Distance
        double distanceX = (targetX - pose.x);
        double distanceY = (targetY - pose.y);
        double distanceToTarget = Math.sqrt(Math.pow((distanceX), 2) + Math.pow(distanceY, 2));

        //Starting values
        if (!isStartRecorded) {
            distanceToTargetStart = distanceToTarget;
            thetaStart = gyroZ;
            isStartRecorded = true;
        }

        //Powers
        double strafe = distanceToTarget * Math.cos(-pose.theta - Math.atan(distanceX / distanceY)) / 24;
        double forward = distanceToTarget * Math.sin(-pose.theta - Math.atan(distanceX / distanceY)) / 24;

        //Threshold values for motor power
        forward = thresholdMotorPower(forward, 0.5);
        strafe = thresholdMotorPower(strafe, 0.5);

        //Adjust for quadrants
        if (pose.y < targetY) {
            forward = -forward;
            strafe = -strafe;
        }

        //Robot orientation vs distance to target (in degrees)
        if (distanceToTarget > distanceToTargetStart * (1 - rotatePercent)) {
            thetaRobot = (1 - (distanceToTarget - distanceToTargetStart * (1 - rotatePercent)) / (distanceToTargetStart * rotatePercent)) * (targetTheta - thetaStart) + thetaStart;
        } else {
            thetaRobot = targetTheta;
        }

        //Positive rotation = clockwise = decrease in theta
        double rotate = (1 / (1 + Math.pow(Math.E, -(0.13 * (gyroZ - thetaRobot))))) - 0.5;

        //Threshold values for motor power
        rotate = thresholdMotorPower(rotate, 0.2);

        drive(forward, strafe, -rotate);
    }

    public boolean isInToleranceOLD(double targetX, double targetY, double toleranceDistance) {
        if (Math.abs(pose.x - targetX) < toleranceDistance && Math.abs(pose.y - targetY) < toleranceDistance) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isInTolerance(double targetX, double targetY, double targetTheta, double toleranceDistance, double toleranceTheta) {
        if (Math.abs(pose.x - targetX) < toleranceDistance && Math.abs(pose.y - targetY) < toleranceDistance && Math.abs(gyroZ - targetTheta) < toleranceTheta) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isStoneCollected(){
        if(dI.getDistance(DistanceUnit.CM) < 15) {
            return true;
        } else {
            return false;
        }
    }

    public double thresholdMotorPower(double power, double threshold) {
        if (power > 1) {
            return 1;
        } else if (power < -1) {
            return -1;
        } else if (power < threshold && power > 0) {
            return threshold;
        } else if (power > -threshold && power < 0) {
            return -threshold;
        } else {
            return power;
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
        mIL.setPower(-power);
        mIR.setPower(power);
        sIL.setPower(power);
        sIR.setPower(-power);
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
}