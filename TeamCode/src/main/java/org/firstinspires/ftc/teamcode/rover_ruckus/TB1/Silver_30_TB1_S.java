package org.firstinspires.ftc.teamcode.rover_ruckus.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Silver_30_TB1_S", group = "Autonomous")
@Disabled
public class Silver_30_TB1_S extends AutoBaseTB1 {

    steps CURRENT_STEP = steps.SCAN_MINERALS;
    double rightSampleCM = 0;
    double threshold = 3;

    final double rightXpos = 94;//dR
    final double rightYpos = 82;//dF

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        defineComponents();
        initVuforia();
        initTfod();
        calibrateGyro();
        activateTfod();

        telemetry.clear();

        //Loop during init to scan for minerals
        while (!opModeIsActive() && !isStopRequested()) {
            getGoldPos();

            telemetry.addData("Gold position: ", goldPos);
            telemetry.update();
        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double dFCM = dF.getDistance(DistanceUnit.CM);
            double dRCM = dR.getDistance(DistanceUnit.CM);
            double gyroZ = modernRoboticsI2cGyro.getIntegratedZValue();

            telemetry.addData("Current Step: ", CURRENT_STEP);
            telemetry.addData("Gold position: ", goldPos);
            telemetry.addData("dFCM: ", String.format("%.01f cm", dFCM));
            telemetry.addData("dRCM: ", String.format("%.01f cm", dRCM));
            telemetry.addData("Gyro: ", String.format("%.01f deg", gyroZ));
            telemetry.addData("Runtime: ", String.format("%.01f sec", runtime.seconds()));
            telemetry.update();

            switch (CURRENT_STEP) {

                case SCAN_MINERALS:
                    if (goldPos != 0) {
                        if(goldPos == 1) {
                            rightSampleCM = 120;
                        }
                        if(goldPos == 2) {
                            rightSampleCM = 94;
                        }
                        if(goldPos == 3) {
                            rightSampleCM = 68;
                        }
                        stopDrivetrain();
                        runtime.reset();
                        CURRENT_STEP = steps.CLEAR_LANDER;
                    } else {
                        getGoldPos();
                    }
                    break;

                case CLEAR_LANDER:
                    if (runtime.seconds() > 1.4) {
                        stopDrivetrain();
                        runtime.reset();
                        CURRENT_STEP = steps.FACE_WALL;
                    } else {
                        driveForward(0.4);
                    }
                    break;

                case FACE_WALL:
                    if (gyroZ < -43) {
                        stopDrivetrain();
                        runtime.reset();
                        CURRENT_STEP = steps.ALIGN_RIGHT;
                    } else {
                        turnToFace(-43);
                    }
                    break;


                //Center front 82, right 94
                //Left front 60, right 119
                //Right front 109, right 68

                case ALIGN_RIGHT:
                    if (dRCM < rightSampleCM + threshold && dRCM > rightSampleCM - threshold) {
                        stopDrivetrain();
                        runtime.reset();
                        CURRENT_STEP = steps.KNOCK_SAMPLE;
                    } else {
                        if(dRCM > rightSampleCM) {
                            mFL.setPower(0.1);
                            mFR.setPower(-0.4);
                            mBL.setPower(-0.4);
                            mBR.setPower(0.1);
                        } else {
                            mFL.setPower(-0.1);
                            mFR.setPower(0.4);
                            mBL.setPower(0.4);
                            mBR.setPower(-0.1);
                        }
                    }
                    break;

                case KNOCK_SAMPLE:
                    if (runtime.seconds() > 3) {
                        stopDrivetrain();
                        runtime.reset();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        driveForward(0.2);
                    }
                    break;

                case STOP:
                    stopDrivetrain();
                    break;

            }
        }
        shutdown();
    }
}

