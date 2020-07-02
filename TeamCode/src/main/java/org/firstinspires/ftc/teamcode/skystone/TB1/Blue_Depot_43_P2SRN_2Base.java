package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue_Depot_43_P2SRN_2Base", group = "Autonomous")
@Disabled

public class Blue_Depot_43_P2SRN_2Base extends AutoBaseTB1 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.WEBCAM_CHECK;

        //Init functions
        defineComponents();
        defineWebcam();
        sCS.setPosition(0.58);

        double targetX = 0;
        double targetY = 0;
        double targetTheta = 0;

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

        while (opModeIsActive()) {

            //Update global sensor values
            gyroUpdate();
            updatePoseStrafe();

            telemetry.addData("Current Step", CURRENT_STEP);
            telemetry.addData("Pos x", pose.x);
            telemetry.addData("Pos y", pose.y);
            telemetry.addData("GyroZ", gyroZ);
            //telemetry.addData("sC position", sC.getPosition());
            telemetry.update();

            switch (CURRENT_STEP) {

                //Scan stones 3, 4, and 5
                case WEBCAM_CHECK:
                    if (skystonePos == "left") {
                        CURRENT_STEP = steps.PREP_SKYSTONE_5;
                    } else if (skystonePos == "middle") {
                        CURRENT_STEP = steps.PREP_SKYSTONE_4;
                    } else if (skystonePos == "right") {
                        CURRENT_STEP = steps.PREP_SKYSTONE_6;
                    } else {
                        CURRENT_STEP = steps.PREP_SKYSTONE_6;
                    }
                    break;

                case PREP_SKYSTONE_4:
                    targetX = 29.7;
                    targetY = 10.4;
                    targetTheta = -34.7;

                    if (runtime.seconds() > 1) {
                        sCS.setPosition(0.05);
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_4;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_4:
                    targetX = 38.2;
                    targetY = 2.5;
                    targetTheta = -41.8;

                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || runtime.seconds() > 2) {
                        changeStep();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_4;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case EXIT_SKYSTONE_4:
                    targetX = 28.7;
                    targetY = 1;
                    targetTheta = -60; //-16.6

                    if (pose.x < targetX + 3){
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PREP_SKYSTONE_5:
                    targetX = 29.7;
                    targetY = 17.4;
                    targetTheta = -34.7;

                    if (runtime.seconds() > 1) {
                        sCS.setPosition(0.05);
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_5;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_5:
                    targetX = 38.2;
                    targetY = 12.5;
                    targetTheta = -41.8;

                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || runtime.seconds() > 2) {
                        changeStep();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_5;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case EXIT_SKYSTONE_5:
                    targetX = 28.7;
                    targetY = 7.7;
                    targetTheta = -16.6;

                    if (pose.x < targetX + 3){
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case PREP_SKYSTONE_6:
                    targetX = 32.4;
                    targetY = 24.1;
                    targetTheta = -34.7;

                    if (runtime.seconds() > 1) {
                        sCS.setPosition(0.05);
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_6;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_6:
                    targetX = 38.2;
                    targetY = 19.5;
                    targetTheta = -41.8;

                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || runtime.seconds() > 2) {
                        changeStep();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_6;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case EXIT_SKYSTONE_6:
                    targetX = 28.7;
                    targetY = 16.7;
                    targetTheta = -16.6;

                    if (pose.x < targetX + 3){
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case DRIVE_TO_SKYBRIDGE:
                    targetX = 21.7;
                    targetY = 39.4;
                    targetTheta = -90;

                    if (pose.y > targetY - 4) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.3);
                    }
                    break;

                case DRIVE_TO_FOUNDATION:
                    targetX = 20.4;
                    targetY = 93;
                    targetTheta = -178;

                    //Grab stone
                    setIntakePower(0);
                    sC.setPosition(0.45);

                    if (isInTolerance(targetX, targetY, targetTheta, 2.5, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PREP_PLACEMENT;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PREP_PLACEMENT:
                    targetX = 23;
                    targetY = 93;
                    targetTheta = -180;

                    //Raise mast
                    if(runtime.seconds() < 0.75) {
                        mML.setPower(-0.6);
                        mMR.setPower(-0.6);
                    } else {
                        mML.setPower(-0.2);
                        mMR.setPower(-0.2);
                    }

                    sG.setPosition(0.36);
                    /*
                    if ((isInTolerance(targetX, targetY, targetTheta, 0.5, 3) && runtime.seconds() > 1)){// || runtime.seconds() > 4) {
                        changeStep();
                        CURRENT_STEP = steps.PLACE_SKYSTONE;
                    } else {
                        if (!isInTolerance(targetX, targetY, targetTheta, 0.5, 3)) {
                            moveToPose(targetX, targetY, targetTheta, 0.4);
                        } else {
                            stopDriveTrain();
                        }
                    }
                    */

                    if(pose.x > targetX && runtime.seconds() > 1) {
                        changeStep();
                        CURRENT_STEP = steps.PLACE_SKYSTONE;
                    } else {
                        driveBackward(0.2);
                    }
                    break;

                case PLACE_SKYSTONE:
                    if (runtime.seconds() < 0.5) {
                        mML.setPower(0.1);
                        mMR.setPower(0.1);
                    } else if (runtime.seconds() >= 0.5 && runtime.seconds() < 0.75) {
                        sC.setPosition(0.7);
                    } else if (runtime.seconds() >= 0.75 && runtime.seconds() < 1.25) {
                        mML.setPower(-0.6);
                        mMR.setPower(-0.6);
                        driveForward(0.4);
                    } else {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_BACK;
                    }
                    break;

                case DRIVE_BACK:
                    targetX = 18.7;
                    targetY = 39.4;
                    targetTheta = -90;

                    setIntakePower(0);
                    mML.setPower(0.1);
                    mMR.setPower(0.1);

                    if (pose.y < targetY + 3) {
                        changeStep();
                        if (skystonePos == "left") {
                            CURRENT_STEP = steps.PREP_SKYSTONE_2;
                        } else if (skystonePos == "middle") {
                            CURRENT_STEP = steps.PREP_SKYSTONE_1;
                        } else if (skystonePos == "right") {
                            CURRENT_STEP = steps.PREP_SKYSTONE_3;
                        } else {
                            CURRENT_STEP = steps.PREP_SKYSTONE_3;
                        }
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.3);
                    }
                    break;

                case PREP_SKYSTONE_1:
                    targetX = 19.2;
                    targetY = -19.8;
                    targetTheta = -40.7;

                    //Retract gantry
                    if (runtime.seconds() > 0.75 && runtime.seconds() < 1.5) {
                        mML.setPower(-0.6);
                        mMR.setPower(-0.6);
                        sG.setPosition(1);
                    } else if (runtime.seconds() >= 1.5 && runtime.seconds() < 3.25){
                        mML.setPower(-0.2);
                        mMR.setPower(-0.2);
                    } else {
                        mML.setPower(0.2);
                        mMR.setPower(0.2);
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_1;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.8); //0.4
                    }
                    break;

                case PICK_UP_SKYSTONE_1:
                    targetX = 34.8;
                    targetY = -21.0;
                    targetTheta = -44.8;

                    mML.setPower(0.2);
                    mMR.setPower(0.2);

                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || runtime.seconds() > 2) {
                        changeStep();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_1;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case EXIT_SKYSTONE_1:
                    targetX = 28.7;
                    targetY = -10.4;
                    targetTheta = -60; //-16.6

                    if (pose.x < targetX + 3){
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case DRIVE_TO_SKYBRIDGE_2:
                    targetX = 18;
                    targetY = 39.4;
                    targetTheta = -90;

                    if (pose.y > targetY - 4) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.3);
                    }
                    break;

                case DRIVE_TO_FOUNDATION_2:
                    targetX = 17.4;
                    targetY = 93;
                    targetTheta = -180;

                    //Grab stone
                    setIntakePower(0);
                    sC.setPosition(0.45);

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PREP_PLACEMENT_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case STRAFE_PLACE_2:
                    targetX = 17.4;
                    targetY = 80.1;
                    targetTheta = -180;

                    //Grab stone
                    setIntakePower(0);
                    sC.setPosition(0.45);

                    //Raise mast
                    if(runtime.seconds() < 0.5) {
                        mML.setPower(-0.6);
                        mMR.setPower(-0.6);
                        sG.setPosition(0.36);
                    } else if(runtime.seconds() > 0.5 && runtime.seconds() < 1) {
                        mML.setPower(-0.2);
                        mMR.setPower(-0.2);
                    } else {
                        mML.setPower(0.2);
                        mMR.setPower(0.2);
                    }

                    if(runtime.seconds() > 0.25) {
                        if (isInTolerance(targetX, targetY, targetTheta, 2.5, 1)) {
                            changeStep();
                            CURRENT_STEP = steps.PREP_PLACEMENT_2;
                        } else {
                            moveToPose(targetX, targetY, targetTheta, 0.4);
                        }
                    }
                    break;

                case PREP_PLACEMENT_2:
                    targetX = 23;
                    targetY = 93;
                    targetTheta = -180;

                    //Raise mast
                    if(runtime.seconds() < 1.25) {
                        mML.setPower(-0.6);
                        mMR.setPower(-0.6);
                    } else {
                        mML.setPower(-0.2);
                        mMR.setPower(-0.2);
                    }

                    sG.setPosition(0.36);
                    /*
                    if ((isInTolerance(targetX, targetY, targetTheta, 0.5, 3) && runtime.seconds() > 1)){// || runtime.seconds() > 4) {
                        changeStep();
                        CURRENT_STEP = steps.PLACE_SKYSTONE;
                    } else {
                        if (!isInTolerance(targetX, targetY, targetTheta, 0.5, 3)) {
                            moveToPose(targetX, targetY, targetTheta, 0.4);
                        } else {
                            stopDriveTrain();
                        }
                    }
                    */

                    if(pose.x > targetX && runtime.seconds() > 1.5) {
                        changeStep();
                        CURRENT_STEP = steps.PLACE_SKYSTONE_2;
                    } else {
                        driveBackward(0.2);
                    }
                    break;

                case PLACE_SKYSTONE_2:
                    if (runtime.seconds() < 0.5) {
                        mML.setPower(0.1);
                        mMR.setPower(0.1);
                    } else if (runtime.seconds() >= 0.5 && runtime.seconds() < 0.75) {
                        sC.setPosition(0.7);
                    } else if (runtime.seconds() >= 0.75 && runtime.seconds() < 1.75) {
                        mML.setPower(-0.8);
                        mMR.setPower(-0.8);
                        driveForward(0.2);
                    } else {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    }
                    break;

                case DRIVE_WITH_FOUNDATION:
                    sP.setPosition(0.4);
                    if (pose.x > 3) {
                        drive(1, 0, -0.4);
                        runtime.reset();
                    } else {
                        if (runtime.seconds() < 1) {
                            drive(0.8, 0, -0.4);
                        } else {
                            releaseFoundation();
                            changeStep();
                            CURRENT_STEP = steps.STRAFE_OUT_FOUNDATION;
                        }
                    }
                    break;

                case STRAFE_OUT_FOUNDATION:
                    if(runtime.seconds() > 1.25) {
                        changeStep();
                        CURRENT_STEP = steps.PARK;
                    } else {
                        drive(-0.2, -1, -0.3);
                    }
                    break;

                case PARK:
                    if(gyroZ < -225) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        rotateClockwise(0.8);
                    }
                    break;

                case STOP:
                    mML.setPower(0);
                    mMR.setPower(0);
                    stopDriveTrain();
                    releaseFoundation();
                    setIntakePower(0);
                    break;
            }
        }
        shutdown();
    }
}