package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCameraRotation;

//Name stands for Place 2 Skystones, Reposition, Navigate
@Autonomous(name = "Red_Depot_49_P3SRN", group = "Autonomous")

public class Red_Depot_49_P3SRN extends AutoBaseTB1 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.WEBCAM_CHECK;

        //Init functions
        defineComponents();
        defineWebcam();
        sCS.setPosition(0.58);

        //Position variables
        double targetX = 0;
        double targetY = 0;
        double targetTheta = 0;

        double skystoneTwoXPos = 0;

        //Open and stream webcam
        webcam.openCameraDevice();
        webcam.setPipeline(new DetectSkystones.StageSwitchingPipeline());
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        //Run skystone algorithm during init
        while (!opModeIsActive() && !isStopRequested()) {
            skystonePos = DetectSkystones.getPos();
            telemetry.addData("Skystone position", skystonePos);
            telemetry.update();
        }

        waitForStart();

        //Reset time variables
        runtime.reset(); //Resets during changeStep()
        runtimeTwo.reset(); //Does not reset unless called to

        while (opModeIsActive()) {

            //Update global sensor values
            gyroUpdate();
            updatePoseStrafe();

            //Telemetry
            telemetry.addData("Current Step", CURRENT_STEP);
            telemetry.addData("Pos x", pose.x);
            telemetry.addData("Pos y", pose.y);
            telemetry.addData("GyroZ", gyroZ);
            telemetry.addData("Skystone 2 X", skystoneTwoXPos);
            telemetry.update();

            switch (CURRENT_STEP) {

                //Scan stones 4, 5, and 6
                case WEBCAM_CHECK:
                    CURRENT_STEP = steps.ADD_MOMENTUM;
                    break;

                case ADD_MOMENTUM:
                    if(runtime.seconds() > 0.3) {
                        if (skystonePos == "left") {
                            CURRENT_STEP = steps.PREP_SKYSTONE_1;
                        } else if (skystonePos == "middle") {
                            CURRENT_STEP = steps.PREP_SKYSTONE_2;
                        } else if (skystonePos == "right") {
                            CURRENT_STEP = steps.PREP_SKYSTONE_3;
                        } else {
                            CURRENT_STEP = steps.PREP_SKYSTONE_1;
                        }
                    } else {
                        drive(0.3, 0, 0);
                        sC.setPosition(0.7);
                    }
                    break;

                case PREP_SKYSTONE_1:
                    targetX = 28;
                    targetY = 23;
                    targetTheta = 40;

                    setIntakePower(0);

                    if (isInTolerance(targetX, targetY, targetTheta, 5, 3)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_1;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_1:
                    targetX = 40;
                    targetY = 23;
                    targetTheta = 40;

                    setIntakePower(-1);

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2) || isStoneCollected() || runtime.seconds() > 2) {
                        changeStep();
                        stoneIntakeTime.reset();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_1;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case EXIT_SKYSTONE_1:
                    if(gyroZ > 85) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        drive(-1, 0, 0.6);
                    }
                    break;

                case DRIVE_TO_SKYBRIDGE:
                    targetX = 28;
                    targetY = -40;
                    targetTheta = 90;

                    if (pose.y < targetY + 3) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case DRIVE_TO_FOUNDATION:
                    targetX = 33;
                    targetY = -87;
                    targetTheta = 180;

                    sC.setPosition(0.45); //Open

                    if (runtime.seconds() < 0.5) {
                        runtimeTwo.reset();
                    } else {
                        extendGantry(0.5);
                    }

                    /*if(runtime.seconds() > 0.5 && runtime.seconds() < 1) { //Handle mast raising
                        mML.setPower(-0.6);
                        mMR.setPower(-0.6);
                        sG.setPosition(0.36);
                    } else if(runtime.seconds() > 1) {
                        mML.setPower(-0.2);
                        mMR.setPower(-0.2);
                    }*/

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 3)) {
                        changeStep();
                        CURRENT_STEP = steps.GRAB_FOUNDATION;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case GRAB_FOUNDATION:
                    targetX = 37.5;
                    targetY = -87;
                    targetTheta = 180;

                    extendGantry(0.5);

                    if(pose.x > targetX - 3) {
                        holdFoundation();
                    }

                    if(pose.x > targetX || runtime.seconds() > 1) {
                        changeStep();
                        holdFoundation();
                        CURRENT_STEP = steps.DRIVE_WITH_FOUNDATION;
                    } else {
                        moveToPoseWaypoint(targetX, targetY, targetTheta, 0.1);
                        // driveForwardGyro(-0.5, 180, 2);
                    }
                    break;

                case DRIVE_WITH_FOUNDATION:
                    targetTheta = 170;

                    extendGantry(0.5);

                    if(gyroZ < targetTheta || runtime.seconds() > 0.3) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_WITH_FOUNDATION_2;
                    } else {
                        rotateCounterclockwise(0.8);
                    }
                    break;

                case DRIVE_WITH_FOUNDATION_2:
                    targetX = 23;

                    extendGantry(0.5);
                    dropStoneWhenDone(3);

                    if(pose.x < targetX && runtimeTwo.seconds() > 3.25) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_WITH_FOUNDATION_3;
                    } else {
                        //if(runtime.seconds() < 0.2) {
                            //dropStoneWhenDone(3.25);
                        //} else {
                            drive(0.8, 0, 0);
                        //}
                    }
                    break;

                case DRIVE_WITH_FOUNDATION_3:
                    targetTheta = 105;

                    if(gyroZ < targetTheta) {
                        changeStep();
                        releaseFoundation();
                        CURRENT_STEP = steps.RETURN_SKYBRIDGE_1;
                    } else {
                        rotateCounterclockwise(1);
                    }
                    break;

                case RETURN_SKYBRIDGE_1:
                    targetX = 18;
                    targetY = -30;
                    targetTheta = 90;

                    setIntakePower(1); //Stop intake

                    if (isInTolerance(targetX, targetY, targetTheta, 5, 3)) {
                        changeStep();
                        runtimeTwo.reset();
                        CURRENT_STEP = steps.PREP_SKYSTONE_4;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PREP_SKYSTONE_4:
                    targetX = 20;
                    targetY = -3;
                    targetTheta = 40;

                    if(runtime.seconds() < 0.5) { //Handle mast raising
                        runtimeTwo.reset();
                        /*mML.setPower(-0.6);
                        mMR.setPower(-0.6);
                        if (runtime.seconds() > 0.4) {
                            sG.setPosition(1);
                        }*/
                    } else {
                        retractGantry();
                    }


                    if (isInTolerance(targetX, targetY, targetTheta, 5, 3)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_4;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_4:
                    targetX = 40;
                    targetY = -1;
                    targetTheta = 40;

                    retractGantry();

                    //mML.setPower(0.05); //Mast down
                    //mMR.setPower(0.05);

                    setIntakePower(-1);

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2) || isStoneCollected()) {
                        changeStep();
                        stoneIntakeTime.reset();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_4;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case EXIT_SKYSTONE_4:

                    retractGantry();

                    if(gyroZ > 85) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_2;
                    } else {
                        drive(-1, 0, 0.6);
                    }
                    break;

                case DRIVE_TO_FOUNDATION_2:
                    targetX = 20;
                    targetY = -70;
                    targetTheta = 90;

                    if(runtime.seconds() > 0.5 && runtime.seconds() < 0.8) {
                        sC.setPosition(0.45);
                    } else if(runtime.seconds() > 0.8 && runtime.seconds() < 1.2) {
                        mML.setPower(-0.8);
                        mMR.setPower(-0.8);
                        sG.setPosition(0.36);
                    } else {
                        mML.setPower(-0.05);
                        mMR.setPower(-0.05);
                    }

                    if (pose.y < targetY + 3) {
                        changeStep();
                        CURRENT_STEP = steps.PLACE_SKYSTONE_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PLACE_SKYSTONE_2:

                    if(runtime.seconds() < 0.5) {
                        mML.setPower(0.1);
                        mMR.setPower(0.1);
                    } else if(runtime.seconds() > 0.5 && runtime.seconds() < 1) {
                        sC.setPosition(0.7);
                    } else {
                        mML.setPower(-0.5);
                        mMR.setPower(-0.5);
                        sG.setPosition(1);
                    }

                    if(runtime.seconds() < 1.2) {
                        drive(-0.4, -0.4, 0);
                    } else {
                        drive(0.4, 0.4, 0);
                    }

                    if (runtime.seconds() > 1.5) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    }
                    break;

                case RETURN_SKYBRIDGE_2:
                    targetX = 22;
                    targetY = -25;
                    targetTheta = 90;

                    mML.setPower(0.05);
                    mMR.setPower(0.05);

                    /*if(runtime.seconds() < 0.3) { //Handle mast raising
                        mML.setPower(-0.8);
                        mMR.setPower(-0.8);
                        sG.setPosition(1);
                    } else if(runtime.seconds() > 0.3 && runtime.seconds() < 1) {
                        mML.setPower(-0.2);
                        mMR.setPower(-0.2);
                    } else {
                        mML.setPower(0.1); //Mast down
                        mMR.setPower(0.1);
                    }*/

                    if(isInTolerance(targetX, targetY, targetTheta, 5, 3)) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        if(runtime.seconds() < 1.2) {
                            drive(0.3, 0.1, 0);
                        } else {
                            moveToPose(targetX, targetY, targetTheta, 0.4);
                        }
                    }
                    break;

                case PREP_EXTRA_STONE:
                    if(gyroZ < 70) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_EXTRA_STONE;
                    } else {
                        drive(0, 0, 0.8);
                    }
                    break;

                case PICK_UP_EXTRA_STONE:

                    setIntakePower(-1);

                    if(runtime.seconds() > 2 || dI.getDistance(DistanceUnit.CM) < 15) {
                        changeStep();
                        stoneIntakeTime.reset();
                        CURRENT_STEP = steps.EXIT_EXTRA_STONE;
                    } else {
                        driveForwardGyro(0.4, 70, 2);
                    }
                    break;

                case EXIT_EXTRA_STONE:
                    targetX = 20;
                    targetY = 0;
                    targetTheta = 90;

                    if (isInTolerance(targetX, targetY, targetTheta, 5, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_3;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case DRIVE_TO_FOUNDATION_3:
                    targetX = 25;
                    targetY = -75;
                    targetTheta = 90;

                   if(runtime.seconds() > 1) {
                        if (runtime.seconds() < 1.1) {
                            sC.setPosition(0.45);
                            setIntakePower(0); //Stop intake
                        } else if(runtime.seconds() > 1.4 && runtime.seconds() < 2) { //Handle mast raising
                            mML.setPower(-0.6);
                            mMR.setPower(-0.6);
                            if(runtime.seconds() > 1.8) {
                                sG.setPosition(0.36);
                            }
                        } else if(runtime.seconds() > 2.4 && runtime.seconds() < 2.6) {
                            mML.setPower(0.1);
                            mMR.setPower(0.1);
                        } else {
                            sC.setPosition(0.7);
                        }
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 5, 3) && stoneIntakeTime.seconds() > 4) {
                        changeStep();
                        CURRENT_STEP = steps.PARK;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PARK:
                    targetX = 25;
                    targetY = -30;
                    targetTheta = 90;

                    if(runtime.seconds() < 0.3) { //Handle mast raising
                        mML.setPower(-0.8);
                        mMR.setPower(-0.8);
                        sG.setPosition(1);
                    } else if(runtime.seconds() > 0.3 && runtime.seconds() < 1) {
                        mML.setPower(-0.2);
                        mMR.setPower(-0.2);
                    } else {
                        mML.setPower(0.1); //Mast down
                        mMR.setPower(0.1);
                    }

                    if(isInTolerance(targetX, targetY, targetTheta, 5, 3)) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        if(runtime.seconds() < 1.2) {
                            drive(0.3, 0.1, 0);
                        } else {
                            moveToPose(targetX, targetY, targetTheta, 0.4);
                        }
                    }
                    break;

                case STOP:
                    //Stop powering motors
                    mML.setPower(-0.05);
                    mMR.setPower(-0.05);
                    setIntakePower(0);
                    stopDriveTrain();
                    break;
            }
        }
        shutdown();
    }
}