//Silver_80_TB2_LSCP

//Imports
package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BlueDepot_33_TB1_DP2SN", group = "Autonomous")
@Disabled

public class BlueDepot_33_TB1_DP2SN extends AutoBaseTB1 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_1;
        //Init functions

        defineComponents();
        double targetX = 0;
        double targetY = 0;
        double setTheta = 0;

        webcam.openCameraDevice();
        webcam.setPipeline(new DetectSkystones.StageSwitchingPipeline());
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (!opModeIsActive() && !isStopRequested()) {
            skystonePos = DetectSkystones.getPos();
            telemetry.addData("Skystone position", skystonePos);
            telemetry.update();
        }

        waitForStart();

        //Reset time variables
        runtime.reset();

        //Clear telemetry
        //telemetry.clear();

        while (opModeIsActive()) {

            //Update global sensor values
            odometerUpdate();
            updatePose();
            gyroUpdate();

            telemetry.addData("Current Step", CURRENT_STEP);
            telemetry.update();

            switch (CURRENT_STEP) {
                case DRIVE_TO_SKYSTONE_1:
                    targetX = 30; //30
                    targetY = -22; //-5

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        setTheta = pose.theta;
                        CURRENT_STEP = steps.ROTATE_TO_SKYSTONE_1;
                    } else {
                        if(runtime.seconds() > 0.5) {
                            sCS.setPosition(0.05);
                        }
                        moveToPoseOLD(targetX, targetY, 1, true, 2, true);
                        //setIntakePower(-1);
                    }
                    break;

                case ROTATE_TO_SKYSTONE_1:
                    if (gyroZ > -40) {
                        changeStep();
                        setTheta = pose.theta;
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_1;
                    } else {
                        rotateSigmoid(-35);
                    }
                    break;

                case PICK_UP_SKYSTONE_1:
                    if (runtime.seconds() > 2) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_BACK_SKYSTONE_1;
                    } else {
                        driveForwardGyro(0.3, Math.toDegrees(setTheta), 2);
                        setIntakePower(-1);
                    }
                    break;

                case DRIVE_BACK_SKYSTONE_1:
                    if (runtime.seconds() > 0.6){
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        drive(-0.4, 0, 0);
                        setIntakePower(0);
                    }
                    break;

                case STOP:
                    drive(0, 0, 0);
                    setIntakePower(0);
                    break;
            }
        }
        shutdown();
    }
}