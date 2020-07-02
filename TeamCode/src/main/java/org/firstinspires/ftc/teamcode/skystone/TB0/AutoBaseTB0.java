package org.firstinspires.ftc.teamcode.skystone.TB0;

//Imports
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public abstract class AutoBaseTB0 extends BaseClassTB0 {

    //Global variables
    double timeAtStop = 0;

    //Cases for auto steps
    public enum steps {

        DRIVE_TO_SKYSTONE_1,
        FACE_SKYSTONE_1,
        PICK_UP_SKYSTONE_1,
        DRIVE_BACK_SKYSTONE_1,
        ROTATE_TOWARDS_SKYBRIDGE_1,
        DRIVE_TO_FOUNDATION_1,
        POSITION_TO_FOUNDATION_1,
        DELIVER_STONE_1,
        RETURN_SKYBRIDGE_1,
        ROTATE_TOWARDS_SKYSTONE_2,
        DRIVE_TO_SKYSTONE_2,
        PICK_UP_SKYSTONE_2,
        DRIVE_BACK_SKYSTONE_2,
        ROTATE_TOWARDS_SKYBRIDGE_2,
        DRIVE_TO_FOUNDATION_2,
        POSITION_TO_FOUNDATION_2,
        DELIVER_STONE_2,
        RETURN_SKYBRIDGE_2,
        STOP,

        //TESTING
        DRIVE_TO_SKYSTONE,
        ROTATE_TO_LINE,
        DETECT_STONE,
        ROTATE_SKYSTONE

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

    public void gyroAdjust(double power, double degree) {

        if (gyroZ > degree) {
            rotateClockwise(power);
        } else if (gyroZ < degree) {
            rotateCounterclockwise(power);
        } else {
            return;
        }
    }

    //Resets encoderAvg and runtime variables to 0
    public void resetSteps() {
        runtime.reset();
        stopDriveTrain();
    }


    //Sigmoid function for rotating clockwise
    public void rotateSigmoid(double degree) {
        double power = (1 / (1 + Math.pow(Math.E, -(0.07 * (gyroZ - degree))))) - 0.5;
        if (power < 0.1 && power > 0) {
            power = 0.1;
        }
        if (power > -0.1 && power < 0) {
            power = -0.1;
        }
        rotateClockwise(power);
    }


    //Displays all sensor readings and important variables for debugging
    public void sensorTelemetry() {
        gyroUpdate();

        telemetry.addData("Gyro: ", gyroZ);
        telemetry.addData("Runtime: ", String.format("%.01f sec", runtime.seconds()));
        telemetry.addData("mBR ticks", mBR.getCurrentPosition());
        telemetry.update();
    }

    public void moveToPose(double targetX, double targetY, int forward, boolean drive, double rotationRatio, boolean slowToTarget) {

        double distanceX = (targetX - pose.x);
        double distanceY = (targetY - pose.y);

        double tangentOf = (targetY - pose.y) / distanceX;
        double distanceToTarget = Math.sqrt(Math.pow((distanceX), 2) + Math.pow(distanceY, 2));

        double distanceToTheta = (Math.atan(tangentOf) - pose.theta);

        if(distanceToTheta > Math.PI / 2) {
            distanceToTheta -= Math.PI;
        }

        if(distanceToTheta < -Math.PI / 2) {
            distanceToTheta += Math.PI;
        }

        double distanceSpeed;

        if(slowToTarget) {
            distanceSpeed = (2 / (1 + Math.pow(Math.E, -0.07 * distanceToTarget))) - 1;
        } else {
            distanceSpeed = 1;
        }

        double leftSidePower;
        double rightSidePower;

        if(distanceToTheta < 0) {
            if(forward > 0) {
                leftSidePower = distanceSpeed * forward; // + orientationSpeed;
                rightSidePower = distanceSpeed * (Math.cos(distanceToTheta) / rotationRatio) * forward; // - orientationSpeed;
                telemetry.addData("Left Side Dominates", "");
            } else {
                leftSidePower = distanceSpeed * (Math.cos(distanceToTheta) / rotationRatio) * forward; // - orientationSpeed;
                rightSidePower = distanceSpeed * forward; // + orientationSpeed;
                telemetry.addData("Right Side Dominates", "");
            }
        } else {
            if(forward > 0) {
                leftSidePower = distanceSpeed * (Math.cos(distanceToTheta) / rotationRatio) * forward; // - orientationSpeed;
                rightSidePower = distanceSpeed * forward; // + orientationSpeed;
                telemetry.addData("Right Side Dominates", "");
            } else {
                leftSidePower = distanceSpeed * forward; // + orientationSpeed;
                rightSidePower = distanceSpeed * (Math.cos(distanceToTheta) / rotationRatio) * forward; // - orientationSpeed;
                telemetry.addData("Left Side Dominates", "");
            }
        }

        if(drive) {
            mFL.setPower(leftSidePower);
            mFR.setPower(rightSidePower);
            mBL.setPower(leftSidePower);
            mBR.setPower(rightSidePower);
        } else {
            telemetry.addData("Pose X", pose.x);
            telemetry.addData("Pose Y", pose.y);
            telemetry.addData("Distance to Theta", Math.toDegrees(distanceToTheta));
            telemetry.addData("Tan", Math.toDegrees(Math.atan(tangentOf)));
            telemetry.addData("Left Side Power", leftSidePower);
            telemetry.addData("Right Side Power", rightSidePower);
            telemetry.update();
        }

    }

    //Shutdown all processes and stop drivetrain
    public void shutdown() {
        stopDriveTrain();
        stop();
    }

     public void changeStep() {
        runtime.reset();
        timeAtStop = stopTime.seconds();
        drive(0,0,0);
     }

     ////////////////////////////////////////////////////////////////////////////////////////////////









}