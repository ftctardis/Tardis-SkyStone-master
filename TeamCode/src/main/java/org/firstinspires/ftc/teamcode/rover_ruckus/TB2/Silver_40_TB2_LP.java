package org.firstinspires.ftc.teamcode.rover_ruckus.TB2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Silver_40_TB2_LP", group = "Autonomous")
@Disabled

public class Silver_40_TB2_LP extends AutoBaseTB2 {

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
            boolean ssLimit = !mlsSS.getState();

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
                    if (runtime.seconds() > 0.7) {
                        stopDriveTrain();
                        mSS.setPower(0);
                        mA.setPower(0.1);
                        runtime.reset();
                        CURRENT_STEP = steps.DROP_LANDER;
                    } else {
                        mA.setPower(0.5);
                    }
                    break;

                case DROP_LANDER:
                    if ((runtime.seconds() > 4 && ssLimit)) {
                        stopDriveTrain();
                        mSS.setPower(0);
                        runtime.reset();
                        CURRENT_STEP = steps.PARK;
                    } else {
                        mSS.setPower(1);
                    }
                    break;


                case PARK:
                    if (runtime.seconds() > 3) {
                        stopDriveTrain();
                        runtime.reset();
                        CURRENT_STEP = steps.STOP;
                    } else if (runtime.seconds() > 2.7) {
                        sD.setPosition(0.8);
                        driveForward(1);
                    } else {
                        driveForward(0.4);
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