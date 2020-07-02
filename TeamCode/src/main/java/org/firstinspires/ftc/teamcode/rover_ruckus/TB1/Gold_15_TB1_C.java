package org.firstinspires.ftc.teamcode.rover_ruckus.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Gold_15_TB1_C", group = "Autonomous")
@Disabled
public class Gold_15_TB1_C extends AutoBaseTB1 {

    steps CURRENT_STEP = steps.LIFT_ARM;

    @Override
    public void runOpMode() { //Start of the initiation for autonomous

        defineComponents();
        fitInBox();

        waitForStart(); //Waits for start

        runtime.reset();

        while (opModeIsActive()) {

            //double dLCM = dL.getDistance(DistanceUnit.CM);
            double dFCM = dF.getDistance(DistanceUnit.CM);
            double dRCM = dR.getDistance(DistanceUnit.CM);

            //telemetry.addData("dLCM: ", String.format("%.01f cm", dLCM));
            telemetry.addData("Current Step: ", CURRENT_STEP);
            telemetry.addData("dFCM: ", String.format("%.01f cm", dFCM));
            telemetry.addData("dRCM: ", String.format("%.01f cm", dRCM));
            telemetry.update();

            sAngle.setPosition(angleUp);

            switch (CURRENT_STEP) {

                case LIFT_ARM:
                    if(runtime.seconds() > 1) {
                        mA.setPower(0.1);
                        runtime.reset();
                        CURRENT_STEP = steps.DRIVE_TO_WALL;
                    } else {
                        mA.setPower(0.5);
                    }
                    break;

                case DRIVE_TO_WALL:
                    mA.setPower(0.1);
                    if (dFCM < 50) {
                        stopDrivetrain();
                        runtime.reset();
                        CURRENT_STEP = steps.CLAIM;
                    } else {
                        closeGripper();
                        driveForward(0.4);
                    }
                    break;

                case CLAIM:
                    if(runtime.seconds() > 1) {
                        openGripper();
                        runtime.reset();
                        CURRENT_STEP = steps.LEAVE_MARKER;
                    } else {
                        mA.setPower(-0.1);
                    }
                    break;

                case LEAVE_MARKER:
                    if(runtime.seconds() > 1) {
                        mA.setPower(0.1);
                        runtime.reset();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        mA.setPower(0.5);
                    }
                    break;


                case STOP:
                    stopDrivetrain();
                    mA.setPower(.1);
                    break;

            }
        }
    }
}