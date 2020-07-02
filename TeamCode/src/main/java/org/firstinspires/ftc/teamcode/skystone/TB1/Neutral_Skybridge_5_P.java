package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Neutral_Skybridge_5_P", group = "Autonomous")

public class Neutral_Skybridge_5_P extends AutoBaseTB1 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.WAIT;

        //Init functions
        defineComponents();
        sCS.setPosition(0.58);

        int wait = 25;

        ElapsedTime holdTime;
        holdTime = new ElapsedTime();

        runtime.reset();
        holdTime.reset();
        boolean isIncremented = false;


        while (!opModeIsActive() && !isStopRequested()) {

            if (gamepad1.a) {
                if (holdTime.seconds() < 0.75 && !isIncremented && wait < 30) {
                    wait++;
                    isIncremented = true;
                } else if (holdTime.seconds() > 0.75 && wait < 30){
                    if (runtime.seconds() > 0.25) {
                        wait++;
                        runtime.reset();
                    }
                }
            } else if (gamepad1.b) {
                if (holdTime.seconds() < 0.75 && !isIncremented && wait > 0) {
                    wait--;
                    isIncremented = true;
                } else if (holdTime.seconds() > 0.75  && wait > 0){
                    if (runtime.seconds() > 0.25) {
                        wait--;
                        runtime.reset();
                    }
                }
            } else {
                holdTime.reset();
                runtime.reset();
                isIncremented = false;
            }

            telemetry.addLine("Use Gamepad 1 A or B to change wait");
            telemetry.addData("Wait", wait + " sec");
            telemetry.update();
        }

        waitForStart();

        //Reset time variables
        runtime.reset();

        while (opModeIsActive()) {

            //Update global sensor values
            gyroUpdate();
            updatePoseStrafe();

            telemetry.addData("Current Step", CURRENT_STEP);

            if (CURRENT_STEP == steps.WAIT) {
                telemetry.addData("Countdown", wait - runtime.seconds());
            } else {
                telemetry.addLine("Countdown finished.");
            }
            telemetry.update();

            switch (CURRENT_STEP) {

                case WAIT:
                    if (runtime.seconds() > wait) {
                        changeStep();
                        CURRENT_STEP = steps.PARK;
                    } else {
                        stopDriveTrain();
                    }
                    break;


                case PARK:

                    if (runtime.seconds() > 0.5) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        driveForward(0.4);
                    }
                    break;

                case STOP:
                    sCS.setPosition(0.05);
                    stopDriveTrain();
                    releaseFoundation();
                    setIntakePower(0);
                    break;
            }
        }
        shutdown();
    }
}