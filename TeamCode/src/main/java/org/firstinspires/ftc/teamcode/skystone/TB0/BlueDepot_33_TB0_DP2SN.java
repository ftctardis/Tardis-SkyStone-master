//Silver_80_TB2_LSCP

//Imports
package org.firstinspires.ftc.teamcode.skystone.TB0;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "BlueDepot_33_TB0_DP2SN", group = "Autonomous")
@Disabled
public class BlueDepot_33_TB0_DP2SN extends AutoBaseTB0 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_1;
        //Init functions

        defineComponents();
        double targetX = 0;
        double targetY = 0;
        double setTheta = 0;

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Skystone position", skystonePos);
            telemetry.update();
        }

        waitForStart();

        //Reset time variables
        runtime.reset();
        stopTime.reset();

        //Clear telemetry
        //telemetry.clear();

        while (opModeIsActive()) {

            //Update global sensor values
            odometerUpdate();
            updatePose();
            gyroUpdate();

            telemetry.addData("Skystone position", skystonePos);
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);

            //Telemetry for steps and sensor values
            telemetry.addData("Current Step: ", CURRENT_STEP);
            //telemetry.addData("Time at Stop", timeAtStop);
            /*telemetry.addData("Pose X: ", pose.x);
            telemetry.addData("Pose Y: ", pose.y);
            telemetry.addData("Pose Theta: ", pose.theta);
            telemetry.update();
            */
            telemetry.update();

            switch (CURRENT_STEP) {
                case DRIVE_TO_SKYSTONE_1:
                    targetX = 30;
                    targetY = -24;

                    if (isInTolerance(targetX, targetY, 5)) {
                        changeStep();
                        setTheta = pose.theta;
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_1;
                    } else {
                        moveToPose(targetX, targetY, 1, true, 10, true);
                        mIL.setPower(1);
                        mIR.setPower(1);
                    }
                    break;

                case PICK_UP_SKYSTONE_1:
                    if (runtime.seconds() > 1 || d1.getDistance(DistanceUnit.CM) < 15) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_BACK_SKYSTONE_1;
                        mIL.setPower(0.5);
                        mIR.setPower(0.5);
                    } else {
                        driveForwardGyro(0.15, Math.toDegrees(setTheta), 1);
                    }
                    break;

                case DRIVE_BACK_SKYSTONE_1:
                    if (runtime.seconds() > 0.6){
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_SKYBRIDGE_1;
                    } else {
                        drive(-0.4, 0, 0);
                        mA.setTargetPosition(-800);
                        mA.setPower(0.5);
                        sC.setPosition(1);
                    }
                    break;

                case ROTATE_TOWARDS_SKYBRIDGE_1:
                    if (gyroZ < -80) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_1;
                    } else {
                        rotateSigmoid(-80);
                    }
                    break;

                case DRIVE_TO_FOUNDATION_1:
                    targetX = 25;
                    targetY = 35;
                    if (isInTolerance(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.POSITION_TO_FOUNDATION_1;
                    } else {
                        moveToPose(targetX, targetY, -1, true, 8, false);
                        if (d1.getDistance(DistanceUnit.CM) < 15) {
                            mIL.setPower(0);
                            mIR.setPower(0);
                        }
                    }
                    break;

                case POSITION_TO_FOUNDATION_1:
                    targetX = 41;
                    targetY = 60;
                    if (isInTolerance(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.DELIVER_STONE_1;
                    } else {
                        moveToPose(targetX, targetY, -1, true, 8, true);
                        mA.setTargetPosition(3000);
                        mA.setPower(0.5);
                        sC.setPosition(0);
                    }
                    break;

                case DELIVER_STONE_1:
                    if (runtime.seconds() > 0.5) {
                        changeStep();
                        CURRENT_STEP = steps.RETURN_SKYBRIDGE_1;
                    } else {
                        drive(0, 0, 0);
                    }
                    break;

                /*case RETURN_SKYBRIDGE_1:
                    targetX = 32;
                    targetY = 13;

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_SKYSTONE_2;
                    } else {
                        moveToPoseOLD(targetX, targetY, 1, true, 8, false);
                    }
                    break;

                case ROTATE_TOWARDS_SKYSTONE_2:
                    if (gyroZ > -45){
                        changeStep();
                        setTheta = pose.theta;
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_2;
                    } else {
                        rotateSigmoid(-45);
                    }
                    break;

                case PICK_UP_SKYSTONE_2:
                    if (runtime.seconds() > 1 || d1.getDistance(DistanceUnit.CM) < 15) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_BACK_SKYSTONE_2;
                        mIL.setPower(0.5);
                        mIR.setPower(0.5);
                    } else {
                        driveForwardGyro(0.2, Math.toDegrees(setTheta), 1);
                        mIL.setPower(1);
                        mIR.setPower(1);
                    }
                    break;

                case DRIVE_BACK_SKYSTONE_2:
                    if (runtime.seconds() > 0.8){
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_SKYBRIDGE_2;
                    } else {
                        drive(-0.6, 0, 0);
                        mA.setTargetPosition(-800);
                        mA.setPower(0.5);
                        sC.setPosition(1);
                    }
                    break;

                case ROTATE_TOWARDS_SKYBRIDGE_2:
                    if (gyroZ < -80) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_2;
                    } else {
                        rotateSigmoid(-80);
                    }
                    break;

                case DRIVE_TO_FOUNDATION_2:
                    targetX = 28;
                    targetY = 30;
                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.POSITION_TO_FOUNDATION_2;
                    } else {
                        moveToPoseOLD(targetX, targetY, -1, true, 8, false);
                        if (d1.getDistance(DistanceUnit.CM) < 15) {
                            mIL.setPower(0);
                            mIR.setPower(0);
                        }
                    }
                    break;

                case POSITION_TO_FOUNDATION_2:
                    targetX = 45;
                    targetY = 60;
                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.DELIVER_STONE_2;
                    } else {
                        moveToPoseOLD(targetX, targetY, -1, true, 10, true);
                        mA.setTargetPosition(3000);
                        mA.setPower(0.5);
                        sC.setPosition(0);
                    }
                    break;

                case DELIVER_STONE_2:
                    if (runtime.seconds() > 0.5) {
                        changeStep();
                        CURRENT_STEP = steps.RETURN_SKYBRIDGE_2;
                    } else {
                        drive(0, 0, 0);
                    }
                    break;

                case RETURN_SKYBRIDGE_2:
                    targetX = 30;
                    targetY = 40;
                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        moveToPoseOLD(targetX, targetY, 1, true, 10, true);
                    }
                    break;*/

                case STOP:
                    drive(0, 0, 0);
                    mIL.setPower(0);
                    mIR.setPower(0);
                    break;
            }
        }
        shutdown();
    }
}