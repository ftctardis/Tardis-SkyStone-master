package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Rotate_drive", group = "Autonomous")
@Disabled

public class Rotate_drive extends AutoBaseTB1 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.WAIT;

        //Init functions
        defineComponents();
        sCS.setPosition(0.58);

        double targetX;
        double targetY;
        double targetHeading;

        waitForStart();

        //Reset time variables
        runtime.reset();

        while (opModeIsActive()) {

            //Update global sensor values
            updatePoseStrafe();
            gyroUpdate();
            /*
            telemetry.addData("Current Step", CURRENT_STEP);
            telemetry.addData("Pos x", pose.x);
            telemetry.addData("Pos y", pose.y);
            telemetry.update();
            */
            switch (CURRENT_STEP) {

                case WAIT:
                    targetX = 30;
                    targetY = 30;
                    targetHeading = -90;

                    if(Math.abs(pose.x - targetX) < 5 && Math.abs(pose.y - targetY) < 5 && Math.abs(gyroZ - targetHeading) < 5) {
                        runtime.reset();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        moveToPoseUpdates(targetX, targetY, targetHeading, true);
                        sC.setPosition(0);
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