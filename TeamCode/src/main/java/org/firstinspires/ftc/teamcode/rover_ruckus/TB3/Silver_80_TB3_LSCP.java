//Silver_80_TB2_LSCP

//Imports
package org.firstinspires.ftc.teamcode.rover_ruckus.TB3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Silver_80_TB3_LSCP", group = "Autonomous")
@Disabled

public class Silver_80_TB3_LSCP extends AutoBaseTB3 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.SCAN_MINERALS;

        //Init functions
        defineComponents();
        fitInBox();
        initVuforia();
        initTfod();
        activateTfod();

        //Loop during init to scan for minerals
        while (!opModeIsActive() && !isStopRequested()) {
            getGoldPos();
        }

        waitForStart();

        //Reset time variables
        runtime.reset();
        matchTime.reset();

        //Clear telemetry
        telemetry.clear();

        while (opModeIsActive()) {

            //Update global sensor values
            gyroUpdate();
            getEncoderAvg();

            //Updates values for distance sensor and super structure limit switch
            double dFCM = dF.getDistance(DistanceUnit.CM);
            boolean ssLimit = !mlsSS.getState();

            //Telemetry for steps and sensor values
            telemetry.addData("Current Step: ", CURRENT_STEP);
            sensorTelemetry();
            telemetry.update();

            switch (CURRENT_STEP) {

                //Scans minerals for gold position
                case SCAN_MINERALS:
                    if (goldPos != 0) {
                        resetSteps();
                        updateMineralTarget();
                        CURRENT_STEP = steps.DROP_LANDER;
                        //Sets goldPos to 1 if no minerals are detected
                    } else if (runtime.seconds() > 2) {
                        goldPos = 1;
                        runtime.reset();
                        CURRENT_STEP = steps.DROP_LANDER;
                    } else {
                        getGoldPos();
                    }
                    break;

                //Raises super structure to de-latch from lander
                case DROP_LANDER:
                    mA.setPower(0.2);
                    mA.setTargetPosition(300);
                    //Checks for magnetic limit switch
                    if ((runtime.seconds() > 2 && ssLimit)) {
                        mSS.setPower(0);
                        resetSteps();
                        encoderAvg = 0;
                        lowerSSTime = matchTime.seconds();
                        CURRENT_STEP = steps.CLEAR_LANDER;
                    } else {
                        mSS.setPower(1);
                    }
                    break;

                //Drives forwards slightly
                case CLEAR_LANDER:
                    lowerSSAuto();
                    mA.setTargetPosition(armSamplePos);
                    if (encoderAvg > 300) {
                        if (runtime.seconds() > 0.5) {
                            resetSteps();
                            CURRENT_STEP = steps.FACE_MINERAL;
                            break;
                        } else {
                            stopDriveTrain();
                        }
                    } else {
                        driveForwardGyro(0.2, 0, 1);
                        runtime.reset();
                    }
                    break;

                //Turns to face mineral based on goldPos
                case FACE_MINERAL:
                    lowerSSAuto();
                    mA.setTargetPosition(armSamplePos);
                    if (gyroZ > targetFACE_MINERAL - 5 && gyroZ < targetFACE_MINERAL + 5) {
                        if (runtime.seconds() > 0.5) {
                            resetSteps();
                            CURRENT_STEP = steps.SAMPLE;
                            break;
                        } else {
                            stopDriveTrain();
                        }
                    } else {
                        rotateSigmoid(targetFACE_MINERAL);
                    }
                    break;

                //Drives forward using encoders, pushes gold block
                case SAMPLE:
                    lowerSSAuto();
                    mA.setTargetPosition(armSamplePos);
                    if (encoderAvg > targetSAMPLE_Silver) {
                        resetSteps();
                        CURRENT_STEP = steps.POSITION_TO_FACE_WALL;
                        break;
                    } else {
                        driveForwardGyro(0.6, targetFACE_MINERAL, 1);
                    }
                    break;

                //Backs up after sampling in between minerals and lander
                case POSITION_TO_FACE_WALL:
                    mA.setTargetPosition(armSamplePos);
                    if (encoderAvg < -350) {
                        resetSteps();
                        CURRENT_STEP = steps.FACE_WALL;
                        break;
                    } else {
                        if (goldPos == 2) {
                            driveBackward(0.2);
                        } else {
                            driveBackward(0.26);
                        }
                    }
                    break;

                //Turns to face wall (footsteps or rover)
                case FACE_WALL:
                    mA.setTargetPosition(armSamplePos + 100);
                    if (gyroZ > 85) {
                        resetSteps();
                        CURRENT_STEP = steps.DRIVE_TO_WALL;
                    } else {
                        rotateSigmoid(85);
                    }
                    break;

                //Drives just short of wall
                case DRIVE_TO_WALL:
                    mA.setTargetPosition(armSamplePos + 100);
                    if (encoderAvg > targetDRIVE_TO_WALL) {
                        resetSteps();
                        CURRENT_STEP = steps.PREPARE_TO_CLAIM;
                    } else {
                        driveForwardGyro(0.6, 90, 1);
                    }
                    break;

                //Drives in position to claim
                case PREPARE_TO_CLAIM:
                    mA.setTargetPosition(armSamplePos + 100);
                    //Strafes into wall
                    if (runtime.seconds() < 1.5) {
                        drive(0, 0.7, -0.5);
                    } else {
                        //Drive towards wall ~80cm away
                        if (encoderAvg > 300) {
                            //Basically checks for anything in range
                            if (dFCM > 50) {
                                isRobotDetected = false;
                            }
                            //Checks for robot
                            if (isRobotDetected && matchTime.seconds() < 22) {
                                stopDriveTrain();
                            } else {
                                //Drive in range for depot
                                if (dFCM > 65 && dFCM < 115) {
                                    resetSteps();
                                    CURRENT_STEP = steps.FAST_CLAIM;
                                } else if (dFCM < 65) {
                                    //Drives backwards
                                    driveForwardGyro(-0.3, 135, 1);
                                } else {
                                    driveForwardGyro(0.5, 135, 1);
                                }
                            }
                        } else {
                            driveForwardGyro(0.5, 135, 1);
                        }
                    }
                    break;

                //Deploys team marker, claims
                case FAST_CLAIM:
                    if (runtime.seconds() < 0.5) {
                        sD.setPosition(0.8); //Deploy arm
                        mA.setTargetPosition(armSamplePos - 100);
                    } else if (runtime.seconds() > 0.25 && runtime.seconds() < 1.5) {
                        mA.setTargetPosition(700);
                        mEX.setPower(0);
                        verticalSweeper(1);
                    } else {
                        resetSteps();
                        CURRENT_STEP = steps.FACE_CRATER;
                    }
                    break;

                //Turns to face crater
                case FACE_CRATER:
                    //Drops marker
                    if (runtime.seconds() < 1) {
                        mA.setTargetPosition(1100);
                        verticalSweeper(1);
                    }
                    //Drives backwards and into wall
                    if (runtime.seconds() > 0.5 && runtime.seconds() < 1) {
                        drive(-0.6, 0.3, 0);
                    } else if (runtime.seconds() >= 1 && runtime.seconds() < 2) {
                        driveForwardGyro(-0.6, 135, 1);
                    } else {
                        if (gyroZ > 315) {
                            resetSteps();
                            CURRENT_STEP = steps.DRIVE_TO_CRATER;
                        } else {
                            //Rotates towards crater to park
                            rotateSigmoid(316);
                            verticalSweeper(1);
                        }
                    }
                    break;

                //Drives to break plane of crater, parks
                case DRIVE_TO_CRATER:
                    mA.setTargetPosition(1100);
                    if (encoderAvg > targetDRIVE_TO_CRATER) {
                        resetSteps();
                        stopTime = matchTime.seconds();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        driveForwardGyro(0.8, 325, 1);
                    }
                    break;

                //Parks and collects minerals
                case STOP:
                    rotateSigmoid(315);

                    if (runtime.seconds() < 1) {
                        mA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        mA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        mA.setPower(0);
                    } else {
                        mA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        mA.setPower(0.3);
                        mA.setTargetPosition(300);
                    }
                    shutdownTfod();

                    //Only collects minerals for max of 3 sec, stops with 0.5 sec left in auto
                    if (matchTime.seconds() > 29.5) {
                        verticalSweeper(0);
                    } else {
                        if (runtime.seconds() > 3) {
                            verticalSweeper(0);
                        } else {
                            verticalSweeper(-1);
                        }
                    }
                    break;
            }
        }
        shutdown();
    }
}