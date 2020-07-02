package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue_Depot_35_DSRSN", group = "Autonomous")
@Disabled

public class Blue_Depot_35_DSRSN extends AutoBaseTB1 {

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
            telemetry.addData("sC position", sC.getPosition());
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
                    targetY = 8.4;
                    targetTheta = -34.7;

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_4;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_4:
                    targetX = 38.2;
                    targetY = 1.5;
                    targetTheta = -41.8;

                    setIntakePower(-1);

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
                    targetTheta = -16.6;

                    if (runtime.seconds() < 0.5) {
                        setIntakePower(-1);
                    } else {
                        setIntakePower(0);
                    }

                    if (pose.x < targetX + 3){
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PREP_SKYSTONE_5:
                    targetX = 29.7;
                    targetY = 15.4;
                    targetTheta = -34.7;

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_5;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_5:
                    targetX = 38.2;
                    targetY = 7.5;
                    targetTheta = -41.8;

                    setIntakePower(-1);

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

                    if (runtime.seconds() < 0.5) {
                        setIntakePower(-1);
                    } else {
                        setIntakePower(0);
                    }

                    if (pose.x < targetX + 3){
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case PREP_SKYSTONE_6:
                    targetX = 32.4; //32.2
                    targetY = 23.1; //23
                    targetTheta = -34.7; //-45

                    /*if (runtime.seconds() > 1) {
                        sCS.setPosition(0.05);
                    }*/

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_6;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_6:
                    targetX = 38.2; //39.7
                    targetY = 19.5; //16.2
                    targetTheta = -41.8; //-56.4

                    //sC.setPosition(0.7);
                    setIntakePower(-1);

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

                    if (runtime.seconds() < 0.5) {
                        setIntakePower(-1);
                    } else {
                        setIntakePower(0);
                    }

                    if (pose.x < targetX + 3){//isInTolerance(targetX, targetY, targetTheta, 3, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case DRIVE_TO_SKYBRIDGE:
                    targetX = 22.7;
                    targetY = 39.4; //37.4
                    targetTheta = -90;

                    /*if (runtime.seconds() < 0.5){
                        setIntakePower(1);
                    } else {
                        setIntakePower(-1);
                    }*/

                    if (pose.y > targetY - 3) {//isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.3);
                    }
                    break;

                case DRIVE_TO_FOUNDATION:
                    targetX = 18.4;
                    targetY = 89.2;
                    targetTheta = -180;

                    //Stop intake and grab stone
                    setIntakePower(0);
                    //sC.setPosition(0.45);

                    if (isInTolerance(targetX, targetY, targetTheta, 2.5, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.GRAB_FOUNDATION;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case GRAB_FOUNDATION:
                    targetX = 30; //32
                    targetY = 89.2;
                    targetTheta = -180;

                    //Raise mast
                    /*if(runtime.seconds() < 1) {
                        mML.setPower(-0.6);
                        mMR.setPower(-0.6);
                    } else {
                        mML.setPower(-0.2);
                        mMR.setPower(-0.2);
                    }*/

                    if (pose.x > targetX) {//isInTolerance(targetX, targetY, targetTheta, 3, 2) && runtime.seconds() > 1) {
                        holdFoundation();
                        changeStep();
                        CURRENT_STEP = steps.REPOSITION_FOUNDATION_1;
                    } else {
                        driveForwardGyro(-0.4, -180, 2);
                        //moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case REPOSITION_FOUNDATION_1:

                    targetX = 18.9;
                    targetY = 64.9;
                    targetTheta = -90;

                    /*sG.setPosition(0.36);
                    mML.setPower(-0.2);
                    mMR.setPower(-0.2);*/
                    holdFoundation();

                    if (runtime.seconds() < 0.5) {
                        stopDriveTrain();
                    } else {
                        if (pose.x < targetX + 2 && pose.y < targetY + 2 || runtime.seconds() > 6) {
                            changeStep();
                            CURRENT_STEP = steps.REPOSITION_FOUNDATION_2;
                        } else {
                            moveToPoseWaypoint(targetX, targetY, targetTheta, 0.8);
                        }
                    }
                    break;


                case REPOSITION_FOUNDATION_2:
                    /*
                    targetX = 25.6;
                    targetY = 83.1;
                    targetTheta = -90;

                    mML.setPower(0.1);
                    mMR.setPower(0.1);

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.8);
                    }
                    break;
                    */

                    if (gyroZ < -90) {
                        changeStep();
                        CURRENT_STEP = steps.REPOSITION_FOUNDATION_3;
                    } else {
                        rotateCounterclockwise(0.6);
                    }

                    break;

                case REPOSITION_FOUNDATION_3:

                    targetX = 25.6;
                    targetY = 83.1;
                    targetTheta = -90;

                    if (pose.y > targetY - 10) {
                        if (runtime.seconds() > 1) {
                            changeStep();
                            CURRENT_STEP = steps.PREP_DEPOSIT;
                        } else {
                            driveForwardGyro(-0.4, -90, 2);
                        }
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.8);
                        runtime.reset();
                    }
                    break;


                case PREP_DEPOSIT:
                    targetX = 17.3;
                    targetY = 62;
                    targetTheta = 0;

                    releaseFoundation();

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.DEPOSIT;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case DEPOSIT:

                    if (runtime.seconds() > 1.5) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_BACK;
                    } else {
                        stopDriveTrain();
                        setIntakePower(1);
                    }
                    break;

                case DRIVE_BACK:
                    targetX = 18.7;
                    targetY = 39.4;
                    targetTheta = -90;

                    setIntakePower(0);

                    if (pose.y < targetY + 3) {//isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
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
                    targetX = 25.3;
                    targetY = -11.9;
                    targetTheta = -90;

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        if (runtime.seconds() < 0.5) {
                            stopDriveTrain();
                        } else {
                            changeStep();
                            CURRENT_STEP = steps.BUMP_SKYSTONE_1;
                        }
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                        runtime.reset();
                    }
                    break;

                case BUMP_SKYSTONE_1:
                    targetX = 37.3;
                    targetY = -13.9;
                    targetTheta = -90;

                    sCS.setPosition(0.05);

                    if (pose.x > targetX){//isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_1;
                    } else {
                        driveRightGyro(-0.4, -90, 2);
                        //moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_1:
                    targetX = 38.8;
                    targetY = -17.2;
                    targetTheta = -90;

                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (pose.y < targetY){//isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || runtime.seconds() > 2) {
                        changeStep();
                        CURRENT_STEP = steps.PREP_SKYBRIDGE_2;
                    } else {
                        driveForwardGyro(0.4, -90, 2);
                        //moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PREP_SKYSTONE_2:
                    targetX = 25.3;
                    targetY = -5.9;
                    targetTheta = -90;

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        if (runtime.seconds() < 0.5) {
                            stopDriveTrain();
                        } else {
                            changeStep();
                            CURRENT_STEP = steps.BUMP_SKYSTONE_2;
                        }
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                        runtime.reset();
                    }
                    break;

                case BUMP_SKYSTONE_2:
                    targetX = 37.3;
                    targetY = -7.9;
                    targetTheta = -90;

                    sCS.setPosition(0.05);

                    if (pose.x > targetX){//isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_2;
                    } else {
                        driveRightGyro(-0.4, -90, 2);
                        //moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_2:
                    targetX = 38.8;
                    targetY = -11.2;
                    targetTheta = -90;

                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (pose.y < targetY){//isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || runtime.seconds() > 2) {
                        changeStep();
                        CURRENT_STEP = steps.PREP_SKYBRIDGE_2;
                    } else {
                        driveForwardGyro(0.4, -90, 2);
                        //moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case PREP_SKYSTONE_3:
                    targetX = 25.3;
                    targetY = 4.9;
                    targetTheta = -90;

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        if (runtime.seconds() < 0.5) {
                            stopDriveTrain();
                        } else {
                            changeStep();
                            CURRENT_STEP = steps.BUMP_SKYSTONE_3;
                        }
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                        runtime.reset();
                    }
                    break;

                case BUMP_SKYSTONE_3:
                    targetX = 37.3;
                    targetY = 4.9;
                    targetTheta = -90;

                    sCS.setPosition(0.05);

                    if (pose.x > targetX){//isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_3;
                    } else {
                        driveRightGyro(-0.4, -90, 2);
                        //moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_3:
                    targetX = 38.8;
                    targetY = -1.2;
                    targetTheta = -90;

                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (pose.y < targetY){//isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || runtime.seconds() > 2) {
                        changeStep();
                        CURRENT_STEP = steps.PREP_SKYBRIDGE_2;
                    } else {
                        driveForwardGyro(0.4, -90, 2);
                        //moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PREP_SKYBRIDGE_2:
                    targetX = 25.3;

                    if (pose.x < targetX){//isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE_2;
                    } else {
                        driveRightGyro(0.4, -90, 2);
                        //moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case DRIVE_TO_SKYBRIDGE_2:
                    targetX = 21.7;
                    targetY = 54.4;
                    targetTheta = -90;

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PARK;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.3);
                    }
                    break;

                case PARK:
                    targetX = 21.7;
                    targetY = 42.4;
                    targetTheta = -90;

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.3);
                    }
                    break;

                case STOP:
                    sC.setPosition(0.7);
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