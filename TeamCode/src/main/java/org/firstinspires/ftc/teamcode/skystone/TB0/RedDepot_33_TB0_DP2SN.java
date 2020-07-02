//Silver_80_TB2_LSCP

//Imports
package org.firstinspires.ftc.teamcode.skystone.TB0;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "RedDepot_33_TB0_DP2SN", group = "Autonomous")
@Disabled
public class RedDepot_33_TB0_DP2SN extends AutoBaseTB0 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_1;
        //Init functions
        defineComponents();
        int targetX = 0;
        int targetY = 0;
        double targetTheta = 0;

        waitForStart();

        //Reset time variables
        runtime.reset();

        //Clear telemetry
        telemetry.clear();

        while (opModeIsActive()) {

            //Update global sensor values
            odometerUpdate();
            updatePoseStrafe();
            gyroUpdate();

            //Telemetry for steps and sensor values
            /*
            telemetry.addData("Current Step: ", CURRENT_STEP);
            telemetry.addData("Pose X: ", pose.x);
            telemetry.addData("Pose Y: ", pose.y);
            telemetry.addData("Pose Theta: ", pose.theta);
            telemetry.update();
            */

            switch (CURRENT_STEP) {
                case DRIVE_TO_SKYSTONE_1:
                    targetX = 20;
                    targetY = 10;

                    if(Math.abs(pose.x - targetX) < 5 && Math.abs(pose.y - targetY) < 5) {
                        runtime.reset();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_1;
                    } else {
                        moveToPose(targetX, targetY, 1, true, 6, false);
                        mIL.setPower(1);
                        mIR.setPower(1);
                    }
                    break;

                case PICK_UP_SKYSTONE_1:
                    if(d1.getDistance(DistanceUnit.CM) < 15) {
                        runtime.reset();
                        mIL.setPower(0);
                        mIR.setPower(0);
                        CURRENT_STEP = steps.STOP;
                    } else {
                        if(runtime.seconds() < 1) {
                            driveForwardGyro(0.2, 25, 0.1);
                        } else {
                            drive(0, 0, 0);
                        }
                    }
                    break;

                case DRIVE_BACK_SKYSTONE_1:
                    if (runtime.seconds() > 0.6){
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_SKYBRIDGE_1;
                    } else {
                        drive(-0.4, 0, 0);
                        mA.setTargetPosition(500);
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
                    targetX = 17;
                    targetY = -50;

                    if(Math.abs(pose.x - targetX) < 5 && Math.abs(pose.y - targetY) < 5) {
                        runtime.reset();
                        CURRENT_STEP = steps.POSITION_TO_FOUNDATION_1;
                    } else {
                        moveToPose(targetX, targetY, -1, true, 6, false);
                    }
                    break;

                case POSITION_TO_FOUNDATION_1:
                    targetX = 32;
                    targetY = -69;

                    if(Math.abs(pose.x - targetX) < 5 && Math.abs(pose.y - targetY) < 5) {
                        runtime.reset();
                        CURRENT_STEP = steps.DELIVER_STONE_1;
                    } else {
                        moveToPose(targetX, targetY, -1, true, 6, true);
                        mA.setTargetPosition(1400);
                        mA.setPower(0.5);
                        sC.setPosition(0);
                    }
                    break;

                case DELIVER_STONE_1:
                    if(runtime.seconds() > 1) {
                        runtime.reset();
                        CURRENT_STEP = steps.RETURN_SKYBRIDGE_1;
                    } else {
                        drive(0, 0, 0);
                    }
                    break;

                case RETURN_SKYBRIDGE_1:
                    targetX = 17;
                    targetY = -10;

                    if(Math.abs(pose.x - targetX) < 5 && Math.abs(pose.y - targetY) < 5) {
                        runtime.reset();
                        CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_2;
                    } else {
                        moveToPose(targetX, targetY, 1, true, 6, false);
                    }
                    break;

                case DRIVE_TO_SKYSTONE_2:
                    targetX = 30;
                    targetY = 0;

                    if(Math.abs(pose.x - targetX) < 5 && Math.abs(pose.y - targetY) < 5) {
                        runtime.reset();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_2;
                    } else {
                        moveToPose(targetX, targetY, 1, true, 6, false);
                    }
                    break;

                case PICK_UP_SKYSTONE_2:
                    if(runtime.seconds() > 1) {
                        runtime.reset();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_2;
                    } else {
                        drive(0, 0, 0);
                    }
                    break;

                case DRIVE_TO_FOUNDATION_2:
                    targetX = 17;
                    targetY = -50;

                    if(Math.abs(pose.x - targetX) < 5 && Math.abs(pose.y - targetY) < 5) {
                        runtime.reset();
                        CURRENT_STEP = steps.POSITION_TO_FOUNDATION_2;
                    } else {
                        moveToPose(targetX, targetY, -1, true, 6, false);
                    }
                    break;

                case POSITION_TO_FOUNDATION_2:
                    targetX = 32;
                    targetY = -69;

                    if(Math.abs(pose.x - targetX) < 5 && Math.abs(pose.y - targetY) < 5) {
                        runtime.reset();
                        CURRENT_STEP = steps.DELIVER_STONE_2;
                    } else {
                        moveToPose(targetX, targetY, -1, true, 6, true);
                        mA.setTargetPosition(1400);
                        mA.setPower(0.5);
                        sC.setPosition(0);
                    }
                    break;

                case DRIVE_BACK_SKYSTONE_2:
                    if (runtime.seconds() > 0.8){
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_SKYBRIDGE_2;
                    } else {
                        drive(-0.6, 0, 0);
                        mA.setTargetPosition(500);
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


                case DELIVER_STONE_2:
                    if(runtime.seconds() > 1) {
                        runtime.reset();
                        CURRENT_STEP = steps.RETURN_SKYBRIDGE_2;
                    } else {
                        drive(0, 0, 0);
                    }
                    break;

                case RETURN_SKYBRIDGE_2:
                    targetX = 17;
                    targetY = -40;

                    if(Math.abs(pose.x - targetX) < 5 && Math.abs(pose.y - targetY) < 5) {
                        runtime.reset();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        moveToPose(targetX, targetY, 1, true, 6, true);
                    }
                    break;

                case STOP:
                    drive(0, 0, 0);
                    break;
            }
        }
        shutdown();
    }
}