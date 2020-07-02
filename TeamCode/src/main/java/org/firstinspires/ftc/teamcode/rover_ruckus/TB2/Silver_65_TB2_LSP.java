package org.firstinspires.ftc.teamcode.rover_ruckus.TB2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Silver_65_TB2_LSP", group = "Autonomous")
@Disabled

public class Silver_65_TB2_LSP extends AutoBaseTB2 {

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

            double dFCMCurr = dF.getDistance(DistanceUnit.CM);
            double dRCM = dR.getDistance(DistanceUnit.CM);
            double gyroZ = angles.firstAngle;
            double mFRTicks = mFR.getCurrentPosition();
            double mBLTicks = mBL.getCurrentPosition();
            boolean ssLimit = !mlsSS.getState();

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
                    if (gyroZ > 43 && gyroZ < 50) {
                        if (runtime.seconds() > 0.5) {
                            resetSteps();
                            CURRENT_STEP = steps.DRIVE_TO_WALL;
                        } else {
                            stopDriveTrain();
                        }
                    } else if (gyroZ > 50) {
                        rotateClockwise(0.15);
                    } else {
                        rotateSigmoid(45);
                    }
                    break;

                case DRIVE_TO_WALL:
                    if (((dFCMCurr < targetDRIVE_TO_WALL && dFCMCurr > targetDRIVE_TO_WALL - 4) || goldPos == 3)) {
                        if (runtime.seconds() > 0.5 || goldPos == 3) {
                            resetSteps();
                            CURRENT_STEP = steps.STRAFE_TO_GOLD;
                        } else {
                            stopDriveTrain();
                        }
                    } else if (dFCMCurr < targetDRIVE_TO_WALL - 4) {
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
                        CURRENT_STEP = steps.FACE_CRATER;
                    } else {
                        driveRightGyro(0.5, 45, 1);
                    }
                    break;

                case FACE_CRATER:
                    if (gyroZ < 10) {
                        resetSteps();
                        CURRENT_STEP = steps.PARK;
                    } else {
                        rotateClockwise(0.2);
                    }
                    break;

                case PARK:
                    if (runtime.seconds() > 1) {
                        stopDriveTrain();
                        runtime.reset();
                        CURRENT_STEP = steps.STOP;
                        stopTime = matchTime.seconds();
                    } else {
                        sD.setPosition(0.8);
                        driveForward(0.5);
                        mA.setPower(-0.1);
                    }
                    break;

                case STOP:
                    stopDriveTrain();
                    mA.setPower(0.1);
                    shutdownTfod();
                    break;
            }
            dFCMPrev = dFCMPrev;
        }
        shutdown();
    }
}