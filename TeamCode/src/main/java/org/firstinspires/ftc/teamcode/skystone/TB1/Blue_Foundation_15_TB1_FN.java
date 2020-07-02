package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Blue_Foundation_15_TB1_FN", group = "Autonomous")
@Disabled

public class Blue_Foundation_15_TB1_FN extends AutoBaseTB1 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.WAIT;

        //Init functions
        defineComponents();
        sCS.setPosition(0.58);

        double targetX = 0;
        double targetY = 0;

        waitForStart();

        //Reset time variables
        runtime.reset();

        while (opModeIsActive()) {

            //Update global sensor values
            updatePoseStrafe();
            gyroUpdate();

            telemetry.addData("Current Step", CURRENT_STEP);
            telemetry.addData("Pos x", pose.x);
            telemetry.addData("Pos y", pose.y);
            telemetry.update();

            switch (CURRENT_STEP) {

                case WAIT:
                    if(runtime.seconds() > 20) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION;
                    }
                    break;

                case DRIVE_TO_FOUNDATION_1:
                    targetX = -15;

                    if (pose.x < targetX) {
                        changeStep();
                        CURRENT_STEP = steps.STRAFE_FOUNDATION;
                    } else {
                        drive(-0.3, 0, 0);
                    }
                    break;

                case STRAFE_FOUNDATION:
                    targetY = -9;

                    if (pose.y < targetY) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_2;
                    } else {
                        driveRightGyro(0.3, 0, 2);
                    }
                    break;

                case DRIVE_TO_FOUNDATION_2:
                    targetX = -34;

                    if (pose.x < targetX) {
                        changeStep();
                        CURRENT_STEP = steps.FINGERS_DOWN;
                    } else {
                        drive(-0.3, 0, 0);
                    }
                    break;

                case FINGERS_DOWN:
                    if(runtime.seconds() > 0.5) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_WITH_FOUNDATION;
                    } else {
                        holdFoundation();
                    }
                    break;

                case DRIVE_WITH_FOUNDATION:
                    if (runtime.seconds() > 3) {
                        changeStep();
                        releaseFoundation();
                        CURRENT_STEP = steps.PARK;
                    } else {
                        drive(0.4, 0, 0);
                    }
                    break;

                case PARK:
                    if (runtime.seconds() > 2) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        driveRightGyro(-0.8, 0, 2);
                    }
                    break;

                case STOP:
                    drive(0, 0, 0);
                    sCS.setPosition(0.05);
                    setIntakePower(0);
                    break;

            }
        }
        shutdown();
    }
}