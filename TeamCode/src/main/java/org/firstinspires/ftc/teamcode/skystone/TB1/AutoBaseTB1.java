package org.firstinspires.ftc.teamcode.skystone.TB1;

//Imports

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraBase;
import org.openftc.easyopencv.OpenCvCameraFactory;

public abstract class AutoBaseTB1 extends BaseClassTB1 {

    //Global variables
    double timeAtStop = 0;

    public void defineWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

    }

    //Drives forward while correcting to face designated gyro heading
    public void driveForwardGyro(double power, double degree, double tolerance) {
        if (gyroZ + tolerance < degree) {
            mFL.setPower(power);
            mFR.setPower(power + 0.3);
            mBL.setPower(power);
            mBR.setPower(power + 0.3);
        } else if (gyroZ - tolerance > degree) {
            mFL.setPower(power + 0.3);
            mFR.setPower(power);
            mBL.setPower(power + 0.3);
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

    public void extendGantry(double seconds) {

        sG.setPosition(0.87); //0.36, 0.99

        if (runtimeTwo.seconds() < seconds) {
            mML.setPower(-0.8);
            mMR.setPower(-0.8);
        } else if (runtimeTwo.seconds() >= seconds && runtimeTwo.seconds() < seconds + 1) {
            mML.setPower(-0.2);
            mMR.setPower(-0.2);
        } else {
            mML.setPower(0.1);
            mMR.setPower(0.1);
        }
    }

    public void retractGantry() {

        sG.setPosition(0.01); //1

        if (runtimeTwo.seconds() < 0.5) {
            mML.setPower(-1);
            mMR.setPower(-1);
        } else if (runtimeTwo.seconds() >= 0.5 && runtimeTwo.seconds() < 1.5) {
            mML.setPower(-0.2);
            mMR.setPower(-0.2);
        } else {
            mML.setPower(0.2);
            mMR.setPower(0.2);
        }
    }

    public void dropStoneWhenDone(double seconds) {

        if (runtimeTwo.seconds() > seconds && runtimeTwo.seconds() < seconds + 0.25) {
            sC.setPosition(0.7);
        } else if (runtimeTwo.seconds() >= seconds + 0.5 && runtimeTwo.seconds() < seconds + 1){
            sC.setPosition(0.7);
            mML.setPower(-0.2);
            mMR.setPower(-0.2);
        } else if (runtimeTwo.seconds() >= seconds + 1 ) {
            mML.setPower(0.3);
            mMR.setPower(0.3);
        }
    }

    //Sigmoid function for rotating clockwise
    public void rotateSigmoid(double degree) {
        double power = (1 / (1 + Math.pow(Math.E, -(0.07 * (gyroZ - degree))))) - 0.5;
        if (power < 0.2 && power > 0) {
            power = 0.2;
        }
        if (power > -0.2 && power < 0) {
            power = -0.2;
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

    public void moveToPoseUpdates(double targetX, double targetY, double gyroLock, boolean drive) {

        double distanceX = (targetX - pose.x);
        double distanceY = (targetY - pose.y);
        double distanceTheta = gyroLock - gyroZ;

        double leftX = distanceX / 20;
        double leftY = -distanceY / 20;
        double rightX = distanceTheta / 180;

        double strafeAssist = 0;

        /*
        if(gyroZ > gyroLock + 2) {
            strafeAssist = (gyroZ - gyroLock) / 25;
        } else if(gyroZ < gyroLock - 2) {
            strafeAssist = (gyroZ - gyroLock) / 25;
        } else {
            strafeAssist = 0;
        }
        */

        double mFLPower = (leftX + leftY) * Math.sin(Math.toRadians(gyroZ) + (Math.PI / 4)) - rightX + strafeAssist;
        double mFRPower = (leftX - leftY) * Math.cos(Math.toRadians(gyroZ) + (Math.PI / 4)) + rightX - strafeAssist;
        double mBLPower = (leftX - leftY) * Math.cos(Math.toRadians(gyroZ) + (Math.PI / 4)) - rightX + strafeAssist;
        double mBRPower = (leftX + leftY) * Math.sin(Math.toRadians(gyroZ) + (Math.PI / 4)) + rightX - strafeAssist;

        telemetry.addData("mFL", mFLPower);
        telemetry.addData("mFR", mFRPower);
        telemetry.addData("mBL", mBLPower);
        telemetry.addData("mBR", mBRPower);
        telemetry.update();

        if (drive) {
            mFL.setPower(mFLPower);
            mFR.setPower(mFRPower);
            mBL.setPower(mBLPower);
            mBR.setPower(mBRPower);
        }
    }

    public void moveToPoseOLD(double targetX, double targetY, int forward, boolean drive, double rotationRatio, boolean slowToTarget) {

        double distanceX = (targetX - pose.x);
        double distanceY = (targetY - pose.y);

        double tangentOf = (targetY - pose.y) / distanceX;
        double distanceToTarget = Math.sqrt(Math.pow((distanceX), 2) + Math.pow(distanceY, 2));

        double distanceToTheta = (Math.atan(tangentOf) - pose.theta);

        if (distanceToTheta > Math.PI / 2) {
            distanceToTheta -= Math.PI;
        }

        if (distanceToTheta < -Math.PI / 2) {
            distanceToTheta += Math.PI;
        }

        double distanceSpeed;

        if (slowToTarget) {
            distanceSpeed = (2 / (1 + Math.pow(Math.E, -0.07 * distanceToTarget))) - 1;
        } else {
            distanceSpeed = 1;
        }

        double leftSidePower;
        double rightSidePower;

        if (distanceToTheta < 0) {
            if (forward > 0) {
                leftSidePower = distanceSpeed * forward; // + orientationSpeed;
                rightSidePower = distanceSpeed * (Math.cos(distanceToTheta) / rotationRatio) * forward; // - orientationSpeed;
                telemetry.addData("Left Side Dominates", "");
            } else {
                leftSidePower = distanceSpeed * (Math.cos(distanceToTheta) / rotationRatio) * forward; // - orientationSpeed;
                rightSidePower = distanceSpeed * forward; // + orientationSpeed;
                telemetry.addData("Right Side Dominates", "");
            }
        } else {
            if (forward > 0) {
                leftSidePower = distanceSpeed * (Math.cos(distanceToTheta) / rotationRatio) * forward; // - orientationSpeed;
                rightSidePower = distanceSpeed * forward; // + orientationSpeed;
                telemetry.addData("Right Side Dominates", "");
            } else {
                leftSidePower = distanceSpeed * forward; // + orientationSpeed;
                rightSidePower = distanceSpeed * (Math.cos(distanceToTheta) / rotationRatio) * forward; // - orientationSpeed;
                telemetry.addData("Left Side Dominates", "");
            }
        }

        if (drive) {
            mFL.setPower(leftSidePower);
            mFR.setPower(rightSidePower);
            mBL.setPower(leftSidePower);
            mBR.setPower(rightSidePower);
        } else {
            telemetry.addData("Pose X", pose.x);
            telemetry.addData("Pose Y", pose.y);
            telemetry.addData("Gyro", gyroZ);
            telemetry.addData("Distance to Theta", Math.toDegrees(distanceToTheta));
            telemetry.addData("Tan", Math.toDegrees(Math.atan(tangentOf)));
            telemetry.addData("Left Side Power", leftSidePower);
            telemetry.addData("Right Side Power", rightSidePower);
            telemetry.update();
        }

    }

    public void moveToPoseStrafeOLD(double targetX, double targetY, int forward, boolean drive, double rotationRatio, boolean slowToTarget) {

        double distanceX = (targetX - pose.x);
        double distanceY = (targetY - pose.y);

        double tangentOf = (targetY - pose.y) / distanceX;
        double distanceToTarget = Math.sqrt(Math.pow((distanceX), 2) + Math.pow(distanceY, 2));

        double distanceToTheta = (Math.atan(tangentOf) - pose.theta);

        //Drive towards target until foot and a half, then go diagonal towards target without rotating
        if (distanceToTarget > 18) {
            moveToPoseOLD(targetX, targetY, forward, drive, rotationRatio, slowToTarget);
        } else {

            //Go faster if further away
            double xPower = (distanceToTarget * Math.cos(distanceToTheta)) / 42; //Drive power
            double yPower = -(distanceToTarget * Math.sin(distanceToTheta)) / 32; //Strafe power //NEGATIVE original

            //Adjust for backwards
            /*if(forward < 0) {
                xPower = -xPower;
                yPower = -yPower;
            }*/ //CODE IN ORIGINAL

            //Adjust if tolerance is missed BLOCK NOT IN ORIGINAL
            if (distanceX < 0) {
                xPower = -xPower;
            }
            if (distanceY < 0) {
                yPower = -yPower;
            }


            //Minimum power is 0.2 for driving
            if (xPower > 0) {
                xPower += 0.4;
            } else {
                xPower -= 0.4;
            }

            //Minimum power is 0.3 for strafing
            if (yPower > 0) {
                yPower += 0.45;
            } else {
                yPower -= 0.45;
            }

            drive(xPower, yPower, 0);
        }
    }

    public void rotateToPose(double targetX, double targetY) {
        double distanceX = (targetX - pose.x);
        double distanceY = (targetY - pose.y);
        double tangentOf = (targetY - pose.y) / distanceX;

        double distanceToTheta = (Math.atan(tangentOf) - pose.theta);

        rotateSigmoid(Math.toDegrees(Math.atan(tangentOf)));

        telemetry.addData("Gyro", gyroZ);
        telemetry.addData("Target", Math.toDegrees(Math.atan(tangentOf)));
        telemetry.update();

    }

    //Shutdown all processes and stop drivetrain
    public void shutdown() {
        stopDriveTrain();
        stop();
    }

    public void changeStep() {
        runtime.reset();
        timeAtStop = stopTime.seconds();
        stopDriveTrain();
        isStartRecorded = false;
    }

    //Cases for auto steps
    public enum steps {

        MOVE_FORWARD,
        FACE_SKYSTONE_1,
        //PICK_UP_SKYSTONE_1,
        //DRIVE_TO_SKYSTONE_1,
        ROTATE_TO_SKYSTONE_1,
        DRIVE_BACK_SKYSTONE_1,
        ROTATE_TOWARDS_SKYBRIDGE_1,
        DRIVE_TO_FOUNDATION_1,
        POSITION_TO_FOUNDATION_1,
        DELIVER_STONE_1,
        RETURN_SKYBRIDGE_1,
        ROTATE_TOWARDS_SKYSTONE_2,
        //DRIVE_TO_SKYSTONE_2,
        //PICK_UP_SKYSTONE_2,
        DRIVE_BACK_SKYSTONE_2,
        ROTATE_TOWARDS_SKYBRIDGE_2,
        //DRIVE_TO_FOUNDATION_2,
        POSITION_TO_FOUNDATION_2,
        DELIVER_STONE_2,
        RETURN_SKYBRIDGE_2,
        STOP,

        //TESTING
        DRIVE_TO_SKYSTONE,
        ROTATE_TO_LINE,
        DETECT_STONE,
        ROTATE_SKYSTONE,

        //FINAL
        ADD_MOMENTUM,
        WEBCAM_CHECK,
        CLEAR_WALL,
        PREP_SKYSTONE_4,
        ROTATE_SKYSTONE_4,
        EXIT_SKYSTONE_4,
        ALIGN_SKYSTONE,
        PICK_UP_SKYSTONE_4,
        PREP_SKYSTONE_5,
        ROTATE_SKYSTONE_5,
        EXIT_SKYSTONE_5,
        PICK_UP_SKYSTONE_5,
        ROTATE_AFTER_SKYSTONE_5,
        PREP_SKYSTONE_6,
        ROTATE_SKYSTONE_6,
        PICK_UP_SKYSTONE_6,
        EXIT_SKYSTONE_6,
        ROTATE_AFTER_SKYSTONE_6,
        ROTATE_TOWARDS_FOUNDATION,
        DRIVE_TO_SKYBRIDGE,
        DRIVE_TO_FOUNDATION,
        PREP_PLACEMENT,
        PLACE_SKYSTONE,
        GRAB_FOUNDATION,
        REPOSITION_FOUNDATION_1,
        REPOSITION_FOUNDATION_2,
        REPOSITION_FOUNDATION_3,
        ROTATE_DEPOSIT,
        PREP_DEPOSIT,
        DEPOSIT,
        ROTATE_BACK,
        DRIVE_BACK,
        DRIVE_TO_SKYSTONE_1,
        PREP_SKYSTONE_1,
        BUMP_SKYSTONE_1,
        ROTATE_SKYSTONE_1,
        PICK_UP_SKYSTONE_1,
        EXIT_SKYSTONE_1,
        DRIVE_TO_SKYSTONE_2,
        PREP_SKYSTONE_2,
        BUMP_SKYSTONE_2,
        ROTATE_SKYSTONE_2,
        PICK_UP_SKYSTONE_2,
        EXIT_SKYSTONE_2,
        DRIVE_TO_SKYSTONE_3,
        PREP_SKYSTONE_3,
        BUMP_SKYSTONE_3,
        ROTATE_SKYSTONE_3,
        PICK_UP_SKYSTONE_3,
        EXIT_SKYSTONE_3,
        ROTATE_TOWARDS_FOUNDATION_2,
        PREP_SKYBRIDGE_2,
        DRIVE_TO_SKYBRIDGE_2,
        PREP_PLACEMENT_2,
        STRAFE_PLACE_2,
        PLACE_SKYSTONE_2,
        DRIVE_TO_FOUNDATION_2,
        ROTATE_DEPOSIT_2,
        DEPOSIT_2,
        DRIVE_BACK_2,
        PREP_STONE_6,
        PICK_UP_STONE_6,
        EXIT_STONE_6,
        PREP_STONE_4,
        PICK_UP_STONE_4,
        EXIT_STONE_4,
        DRIVE_TO_SKYBRIDGE_3,
        DRIVE_TO_FOUNDATION_3,
        PREP_PLACEMENT_3,
        PLACE_STONE_3,
        ROTATE_PARK,
        STRAFE_OUT_FOUNDATION,
        PREP_PARK,
        PARK,
        PREP_EXTRA_STONE,
        PICK_UP_EXTRA_STONE,
        EXIT_EXTRA_STONE,

        WAIT,
        DRIVE_WITH_FOUNDATION,
        DRIVE_WITH_FOUNDATION_2,
        FINGERS_DOWN,
        STRAFE_FOUNDATION,
        STRAFE_WITH_FOUNDATION,
        DRIVE_WITH_FOUNDATION_3

    }
}