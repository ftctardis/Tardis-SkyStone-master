package org.firstinspires.ftc.teamcode.rover_ruckus.TB3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Gold_80_TB3_LSCP", group = "Autonomous")
@Disabled
public class Gold_80_TB3_LSCP extends AutoBaseTB3 {

    boolean turnCheck = false;

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
                    if (encoderAvg > targetSAMPLE) {
                        resetSteps();
                        CURRENT_STEP = steps.PREPARE_TO_CLAIM;
                        break;
                    } else {
                        driveForwardGyro(0.6, targetFACE_MINERAL, 1);
                    }
                    break;

                case PREPARE_TO_CLAIM:
                    mA.setTargetPosition(armSamplePos);
                    if (goldPos == 1) {
                        if (encoderAvg > 700) {
                            resetSteps();
                            CURRENT_STEP = steps.LEFT_PREPARE_TO_CLAIM;
                        } else {
                            drive(0.3, 0, 0.4);
                        }
                    } else if (goldPos == 2) {
                        CURRENT_STEP = steps.CENTER_PREPARE_TO_CLAIM;
                    } else {
                        if (gyroZ > 40) {
                            resetSteps();
                            CURRENT_STEP = steps.RIGHT_PREPARE_TO_CLAIM;
                        } else {
                            drive(-0.3, 0.5, -0.4);
                        }
                    }
                    break;

                case LEFT_PREPARE_TO_CLAIM:
                    mA.setTargetPosition(armSamplePos);
                    if (runtime.seconds() > 1.8) {
                        resetSteps();
                        CURRENT_STEP = steps.ADJUST_DEPOT;
                    } else if (runtime.seconds() < 0.3) {
                        driveBackward(0.3);
                    } else {
                        driveRightGyro(-0.5, -45, 1);
                    }
                    break;

                case CENTER_PREPARE_TO_CLAIM:
                    mA.setTargetPosition(armSamplePos);
                    if (runtime.seconds() > 0.5 && gyroZ < -40) {
                        resetSteps();
                        CURRENT_STEP = steps.ADJUST_DEPOT;
                    } else if (runtime.seconds() < 0.5) {
                        driveBackward(0.3);
                    } else {
                        drive(-0.05, -1, 0.15);
                    }
                    break;

                case RIGHT_PREPARE_TO_CLAIM:
                    mA.setTargetPosition(armSamplePos);
                    if (runtime.seconds() > 1.5) {
                        resetSteps();
                        CURRENT_STEP = steps.RIGHT_PREPARE_TO_CLAIM2;
                    } else {
                        driveRightGyro(0.8, 45, 1);
                    }
                    break;

                case RIGHT_PREPARE_TO_CLAIM2:
                    if (turnCheck && gyroZ < -40) {
                        resetSteps();
                        CURRENT_STEP = steps.ADJUST_DEPOT;
                    } else if (!turnCheck) {
                        driveForwardGyro(0.5, 45, 1);
                        if (dFCM < 70 || (dFCM > 600 && encoderAvg > 1000)) {
                            runtime.reset();
                            turnCheck = true;
                        }
                    } else {
                        drive(-0.05, -0.7, 0.3);
                    }
                    break;

                case ADJUST_DEPOT:
                    mA.setTargetPosition(armSamplePos + 100);
                    //Drive in range for depot
                    if (dFCM > 55 && dFCM < 80) {
                        resetSteps();
                        CURRENT_STEP = steps.FAST_CLAIM;
                    } else if (dFCM <= 55) {
                        //Drives backwards
                        driveForwardGyro(-0.15, -45, 1);
                    } else {
                        driveForwardGyro(0.15, -45, 1);
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
                        driveForwardGyro(-0.6, -45, 1);
                    } else {
                        if (gyroZ < -230) {
                            resetSteps();
                            CURRENT_STEP = steps.DRIVE_TO_CRATER;
                            break;
                        } else {
                            //Rotates towards crater to park
                            rotateSigmoid(-231);
                            verticalSweeper(1);
                        }
                    }
                    break;

                case DRIVE_TO_CRATER:
                    mA.setTargetPosition(1100);
                    if (encoderAvg > targetDRIVE_TO_CRATER_Gold) {
                        resetSteps();
                        stopTime = matchTime.seconds();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        driveForwardGyro(0.8, -230, 1);
                    }
                    break;

                case STOP:
                    rotateSigmoid(-225);

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
