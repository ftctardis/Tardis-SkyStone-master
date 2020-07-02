package org.firstinspires.ftc.teamcode.rover_ruckus.TB2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Gold_45_TB2_LC", group = "Autonomous")
@Disabled

public class Gold_45_TB2_LC extends AutoBaseTB2 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        steps CURRENT_STEP = steps.CLEAR_ARM;

        defineComponents();
        fitInBox();
        telemetry.addData("Ready.", "");
        telemetry.update();

        waitForStart();

        runtime.reset();
        telemetry.clear();

        while (opModeIsActive()) {

            gyroUpdate();

            double dFCM = dF.getDistance(DistanceUnit.CM);
            double dRCM = dR.getDistance(DistanceUnit.CM);
            double gyroZ = angles.firstAngle;
            boolean ssLimit = !mlsSS.getState(); //TODO: From Matt. Remember to use "is" in your booleans!! Also, the "not" is very confusing

            telemetry.addData("Current Step: ", CURRENT_STEP);
            //telemetry.addData("Gold position: ", goldPos);
            telemetry.addData("dFCM: ", String.format("%.01f cm", dFCM));
            telemetry.addData("dRCM: ", String.format("%.01f cm", dRCM));
            telemetry.addData("Gyro: ", String.format("%.01f deg", gyroZ));
            telemetry.addData("SS Limit: ", ssLimit);
            telemetry.addData("Runtime: ", String.format("%.01f sec", runtime.seconds()));
            telemetry.update();

            switch (CURRENT_STEP) {

                case CLEAR_ARM:
                    if (runtime.seconds() > 1) {
                        stopDriveTrain();
                        mSS.setPower(0);
                        mA.setPower(0.05);
                        runtime.reset();
                        CURRENT_STEP = steps.DROP_LANDER;
                    } else {
                        mA.setPower(0.5);
                    }
                    break;

                case DROP_LANDER:
                    if ((runtime.seconds() > 4  && ssLimit)) {
                        stopDriveTrain();
                        mSS.setPower(0);
                        runtime.reset();
                        CURRENT_STEP = steps.DRIVE_TO_DEPOT;
                    } else {
                        mSS.setPower(1);
                    }
                    break;

                case DRIVE_TO_DEPOT:
                    if (runtime.seconds() > 3.5) {
                        stopDriveTrain();
                        sD.setPosition(0.8);
                        runtime.reset();
                        CURRENT_STEP = steps.DROP_ARM;
                    } else {
                        driveForward(0.4);
                    }
                    break;

                case DROP_ARM:
                    if ((runtime.seconds() > 3)) {
                        stopDriveTrain();
                        runtime.reset();
                        CURRENT_STEP = steps.DROP_MARKER;
                    } else {
                        mA.setPower(-0.35);
                    }
                    break;

                case DROP_MARKER:
                    if ((runtime.seconds() > 2)) {
                        stopDriveTrain();
                        verticalSweeper(0);
                        mA.setPower(0.05);
                        runtime.reset();
                        CURRENT_STEP = steps.EXIT_DEPOT;
                    } else {
                        mA.setPower(0.5);
                        verticalSweeper(0.5);
                    }
                    break;

                case EXIT_DEPOT:
                    if (runtime.seconds() > 1) {
                        stopDriveTrain();
                        runtime.reset();
                        CURRENT_STEP = steps.TRY_FOR_SAMPLING;
                    } else {
                        driveBackward(0.4);
                    }
                    break;

                case TRY_FOR_SAMPLING:
                    if (runtime.seconds() > 0.5) {
                        stopDriveTrain();
                        runtime.reset();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        driveRight(0.4);
                    }
                    break;

                case STOP:
                    stopDriveTrain();
                    mA.setPower(.1);
                    break;
            }
        }
        shutdown();
    }
}