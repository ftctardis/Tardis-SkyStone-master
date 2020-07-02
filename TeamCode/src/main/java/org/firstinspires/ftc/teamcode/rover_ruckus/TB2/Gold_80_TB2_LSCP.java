package org.firstinspires.ftc.teamcode.rover_ruckus.TB2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Gold_80_TB2_LSCP", group = "Autonomous")
@Disabled

public class Gold_80_TB2_LSCP extends AutoBaseTB2 {


    boolean turnCheck = false;

    @Override
    //Start of the initiation for autonomous
    public void runOpMode() {

        steps CURRENT_STEP = steps.SCAN_MINERALS;

        //Init functions
        defineComponents();
        fitInBox();
        initVuforia();
        initTfod();
        activateTfod();

        telemetry.clear();

        //Loop during init to scan for minerals
        while (!opModeIsActive() && !isStopRequested()) {
            getGoldPos();
        }

        waitForStart();

        //Reset time variables
        runtime.reset();
        matchTime.reset();
        telemetry.clear();

        while (opModeIsActive()) {

            //Update global sensor value
            gyroUpdate();
            getEncoderAvg();

            double dFCM = dF.getDistance(DistanceUnit.CM);
            boolean ssLimit = !mlsSS.getState();

            //Telemetry for steps and sensors
            telemetry.addData("Current Step: ", CURRENT_STEP);
            sensorTelemetry();
            telemetry.update();

            switch (CURRENT_STEP) {

                case SCAN_MINERALS:
                    if (goldPos != 0) {
                        resetSteps();
                        updateMineralTarget();
                        CURRENT_STEP = steps.DROP_LANDER;
                    } else if (runtime.seconds() > 2) {
                        goldPos = 1;
                        runtime.reset();
                        CURRENT_STEP = steps.DROP_LANDER;
                    } else {
                        getGoldPos();
                    }
                    break;

                case DROP_LANDER:
                    mA.setPower(0.5);
                    mA.setTargetPosition(300);
                    if ((runtime.seconds() > 1 && ssLimit)) {
                        mSS.setPower(0);
                        resetSteps();
                        lowerSSTime = matchTime.seconds();
                        CURRENT_STEP = steps.CLEAR_LANDER;
                    } else {
                        mSS.setPower(1);
                    }
                    break;

                case CLEAR_LANDER:
                    lowerSSAuto();
                    mA.setTargetPosition(armSamplePos);
                    if (encoderAvg > 100 && encoderAvg < 300) {
                        if (runtime.seconds() > 0.25) {
                            resetSteps();
                            CURRENT_STEP = steps.FACE_MINERAL;
                        } else {
                            stopDriveTrain();
                        }
                    } else if (encoderAvg > 300) {
                        driveBackward(0.1);
                        runtime.reset();
                    } else {
                        driveForwardGyro(0.2, 0, 1);
                        runtime.reset();
                    }
                    break;

                case FACE_MINERAL:
                    lowerSSAuto();
                    mA.setTargetPosition(armSamplePos);
                    if (gyroZ > targetFACE_MINERAL - 5 && gyroZ < targetFACE_MINERAL + 5) {
                        if (runtime.seconds() > 0.25) {
                            resetSteps();
                            CURRENT_STEP = steps.SAMPLE;
                        } else {
                            stopDriveTrain();
                        }
                    } else {
                        rotateSigmoid(targetFACE_MINERAL);
                    }
                    break;

                case SAMPLE:
                    lowerSSAuto();
                    mA.setTargetPosition(armSamplePos);
                    if (encoderAvg > targetSAMPLE) {
                        resetSteps();
                        CURRENT_STEP = steps.PREPARE_TO_CLAIM;
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
                            drive(-0.1, 0.2, -0.3);
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
                        drive(-0.05, -0.7, 0.2);
                    }
                    break;

                case RIGHT_PREPARE_TO_CLAIM:
                    mA.setTargetPosition(armSamplePos);
                    if (runtime.seconds() > 1) {
                        resetSteps();
                        CURRENT_STEP = steps.RIGHT_PREPARE_TO_CLAIM2;
                    } else {
                        driveRightGyro(0.4, 45, 1);
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
                    if (dFCM > 75 && dFCM < 115) {
                        resetSteps();
                        CURRENT_STEP = steps.FAST_CLAIM;
                    } else if (dFCM <= 75) {
                        //Drives backwards
                        driveForwardGyro(-0.15, -45, 1);
                    } else {
                        driveForwardGyro(0.15, -45, 1);
                    }
                    break;

                case FAST_CLAIM:
                    if (runtime.seconds() < 0.5) {
                        sD.setPosition(0.8); //Deploy arm
                        mA.setTargetPosition(600);
                    } else if (runtime.seconds() > 0.25 && runtime.seconds() < 1.5) {
                        mA.setTargetPosition(1000);
                        mEX.setPower(0);
                        verticalSweeper(1);
                    } else {
                        resetSteps();
                        CURRENT_STEP = steps.AVOID_MINERAL;
                    }
                    break;

                case AVOID_MINERAL:
                    mA.setTargetPosition(1800);
                    if (encoderAvg < -400) {
                        resetSteps();
                        CURRENT_STEP = steps.FACE_CRATER;
                    } else {
                        drive(-0.6, -0.05, 0);
                    }
                    break;

                case FACE_CRATER:
                    /*if (runtime.seconds() < 1) {
                        mA.setTargetPosition(1800); //1600
                        driveRight(0.2);
                        verticalSweeper(1);
                    }*/
                    if (runtime.seconds() > 0.5 && runtime.seconds() < 1) {
                        drive(-0.6, 0.1, 0);
                    } else {
                        if (gyroZ < -225) {
                            resetSteps();
                            CURRENT_STEP = steps.DRIVE_TO_CRATER;
                        } else {
                            rotateSigmoid(-226);
                            verticalSweeper(1);
                        }
                    }
                    break;

                case DRIVE_TO_CRATER:
                    mA.setTargetPosition(1800);
                    if (encoderAvg > targetDRIVE_TO_CRATER) {
                        resetSteps();
                        stopTime = matchTime.seconds();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        driveForwardGyro(0.8, -225, 1);
                    }
                    break;

                case STOP:
                    rotateSigmoid(-225);
                    mA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    mA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    mA.setPower(0);
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
