package org.firstinspires.ftc.teamcode.rover_ruckus.TB2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Gold_70_TB2_LSC", group = "Autonomous")
@Disabled

public class Gold_70_TB2_LSC extends AutoBaseTB2 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        steps CURRENT_STEP = steps.SCAN_MINERALS;

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

        runtime.reset();
        matchTime.reset();
        telemetry.clear();

        while (opModeIsActive()) {

            gyroUpdate();
            getEncoderAvg();

            double dFCM = dF.getDistance(DistanceUnit.CM);
            double dRCM = dR.getDistance(DistanceUnit.CM);
            double gyroZ = angles.firstAngle;
            double mFRTicks = mFR.getCurrentPosition();
            double mBLTicks = mBL.getCurrentPosition();
            boolean ssLimit = !mlsSS.getState();

            telemetry.addData("Current Step: ", CURRENT_STEP);
            sensorTelemetry();
            telemetry.update();

            //if (CURRENT_STEP != steps.FAST_CLAIM || CURRENT_STEP != steps.EXTENDED_CLAIM || CURRENT_STEP != steps.STOP) {
                //verticalSweeper(-0.1);
            //}

            switch (CURRENT_STEP) {

                case SCAN_MINERALS:
                    if (goldPos != 0) {
                        resetSteps();
                        updateMineralTarget();
                        verticalSweeper(-0.1);
                        CURRENT_STEP = steps.DROP_LANDER;
                    } else if (runtime.seconds() > 2) {
                        goldPos = 1;
                        runtime.reset();
                        verticalSweeper(-0.1);
                        CURRENT_STEP = steps.DROP_LANDER;
                    } else {
                        getGoldPos();
                    }
                    break;

                case DROP_LANDER:
                    if ((runtime.seconds() > 4 && ssLimit)) {
                        mSS.setPower(0);
                        resetSteps();
                        CURRENT_STEP = steps.CLEAR_LANDER;
                    } else {
                        if (runtime.seconds() < 0.7) {
                            mA.setPower(0.5);
                            mSS.setPower(0.5);
                        } else {
                            mA.setPower(0.1);
                            mSS.setPower(1);
                        }
                    }
                    break;

                case CLEAR_LANDER:
                    if (encoderAvg > targetCLEAR_LANDER && encoderAvg < targetCLEAR_LANDER + 300) {
                        if (runtime.seconds() > 0.5) {
                            resetSteps();
                            CURRENT_STEP = steps.FACE_WALL;
                        } else {
                            stopDriveTrain();
                        }
                    } else if (encoderAvg > targetCLEAR_LANDER + 300) {
                        driveBackward(0.1);
                        runtime.reset();
                    } else {
                        driveForwardGyro(0.2, 0, 1);
                        runtime.reset();
                    }
                    break;

                case FACE_WALL:
                    if (gyroZ > 45 && gyroZ < 50) {
                        if (runtime.seconds() > 0.5) {
                            resetSteps();
                            CURRENT_STEP = steps.DRIVE_TO_WALL;
                        } else {
                            stopDriveTrain();
                        }
                    } else if (gyroZ > 50) {
                        rotateClockwise(0.15);
                    }
                    else {
                        rotateSigmoid(47.5);
                    }
                    break;

                case DRIVE_TO_WALL:
                    if ((dFCM < targetDRIVE_TO_WALL && dFCM > targetDRIVE_TO_WALL - 4) || goldPos == 3) {
                        if (runtime.seconds() > 0.5) {
                            resetSteps();
                            CURRENT_STEP = steps.STRAFE_TO_GOLD;
                        } else {
                            stopDriveTrain();
                        }
                    } else if (dFCM < targetDRIVE_TO_WALL - 4) {
                        driveBackward(0.07);
                        runtime.reset();
                    } else {
                        driveDistanceSigmoid(targetDRIVE_TO_WALL, 45, 1);
                        runtime.reset();
                    }
                    break;

                case STRAFE_TO_GOLD:
                    if (encoderAvg < targetSTRAFE_TO_GOLD) {
                        resetSteps();
                        CURRENT_STEP = steps.FACE_DEPOT;
                    } else {
                        driveRight(0.5);
                    }
                    break;

                case FACE_DEPOT:
                    if (gyroZ > 70) {
                        resetSteps();
                        CURRENT_STEP = steps.PREPARE_TO_CLAIM;
                        //stopTime = matchTime.seconds();
                    } else {
                        drive(0, -0.4, -0.25); //Strafes left while rotating counterclockwise
                    }
                    break;

                case PREPARE_TO_CLAIM:
                    if (encoderAvg > targetPREPARE_TO_CLAIM) {
                        resetSteps();
                        CURRENT_STEP = steps.ROTATE_CLAIM;
                    } else {
                        driveForwardGyro(0.4, 90, 1);
                    }
                    break;

                case ROTATE_CLAIM:
                    if (gyroZ < -25) {
                        if (matchTime.seconds() < 26) {
                            resetSteps();
                            CURRENT_STEP = steps.EXTENDED_CLAIM;
                        } else {
                            resetSteps();
                            CURRENT_STEP = steps.FAST_CLAIM;

                        }
                    } else {
                        mFL.setPower(0.4);
                        mBL.setPower(0.4);
                    }
                    break;

                case FAST_CLAIM:
                    if (dFCM < 70) {
                        resetSteps();
                        CURRENT_STEP = steps.STOP;
                        //stopTime = matchTime.seconds();
                    } else if (dFCM < 90) {
                        verticalSweeper(0.5);
                        driveDistanceSigmoid(70, -45, 1);
                    } else {
                        driveDistanceSigmoid(70, -45, 1);
                    }
                    break;

                case EXTENDED_CLAIM:
                    if (dFCM < 70 && runtime.seconds() > 4) {
                        resetSteps();
                        CURRENT_STEP = steps.STOP;
                        //stopTime = matchTime.seconds();
                    }
                    if (dFCM > 70) {
                        driveDistanceSigmoid(70, -45, 1);
                        runtime.reset();
                    } else {

                        stopDriveTrain();

                        if (runtime.seconds() < 1) {
                            sD.setPosition(0.8);
                            mA.setPower(-0.5);
                        } else if (runtime.seconds() > 1 && runtime.seconds() < 2.5){
                            mA.setPower(0.5);
                            verticalSweeper(0.5);
                        } else {
                            mA.setPower(0.1);
                            driveBackward(0.2);
                            verticalSweeper(0.5);
                        }
                    }
                    break;

                case STOP:
                    stopDriveTrain();
                    verticalSweeper(0.5);
                    mA.setPower(0.1);
                    shutdownTfod();
                    break;
            }
        }
        shutdown();
    }
}