package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCameraRotation;

//Name stands for Place 2 Skystones, Reposition, Navigate
@Autonomous(name = "Red_Depot_49_Andrew", group = "Autonomous")

public class Red_Depot_49_Andrew extends AutoBaseTB1 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.WEBCAM_CHECK;

        //Init functions
        defineComponents();
        defineWebcam();

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
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("RuntimeTwo", runtimeTwo.seconds());
            telemetry.addData("Mast power", mML.getPower());
            telemetry.update();

            switch (CURRENT_STEP) {

                //Scan stones 4, 5, and 6
                case WEBCAM_CHECK:
                    CURRENT_STEP = steps.ADD_MOMENTUM;
                    break;

                case ADD_MOMENTUM:
                    if (runtime.seconds() > 0.3) {
                        //Set step from skystone position
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
                    if (gyroZ > 85) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        drive(-1, 0, 0.6);
                    }
                    break;

                case PREP_SKYSTONE_2:
                    targetX = 25.8;
                    targetY = 9;
                    targetTheta = 40;

                    setIntakePower(0);

                    if (isInTolerance(targetX, targetY, targetTheta, 5, 3)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_2:
                    targetX = 41;
                    targetY = 13;
                    targetTheta = 40;

                    setIntakePower(-1);

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2) || isStoneCollected() || runtime.seconds() > 2) {
                        changeStep();
                        stoneIntakeTime.reset();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_1; //Identical step
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
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
                    targetY = -90.5;
                    targetTheta = 180;

                    sC.setPosition(0.45); //Hold stone

                    if (runtime.seconds() < 0.5) {
                        runtimeTwo.reset();
                    } else {
                        extendGantry(0.5);
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 3)) {
                        changeStep();
                        CURRENT_STEP = steps.GRAB_FOUNDATION;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case GRAB_FOUNDATION:
                    targetX = 37.5;
                    targetY = -90.5;
                    targetTheta = 180;

                    extendGantry(0.5);

                    if (pose.x > targetX - 3) {
                        //Grab foundation
                        if (runtime.seconds() < 0.5) {
                            driveBackward(0.15);
                            holdFoundation();

                        } else {
                            changeStep();
                            CURRENT_STEP = steps.DRIVE_WITH_FOUNDATION;
                        }
                    } else {
                        moveToPoseWaypoint(targetX, targetY, targetTheta, 0.1);
                        runtime.reset();
                        // driveForwardGyro(-0.5, 180, 2);
                    }
                    break;

                case DRIVE_WITH_FOUNDATION:
                    targetTheta = 170;

                    extendGantry(0.5);

                    if (gyroZ < targetTheta || runtime.seconds() > 0.3) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_WITH_FOUNDATION_2;
                    } else {
                        rotateCounterclockwise(0.8);
                    }
                    break;

                case DRIVE_WITH_FOUNDATION_2:
                    targetX = 27;

                    extendGantry(0.5);

                    if (pose.x < targetX - 3) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_WITH_FOUNDATION_3;
                    } else {
                        drive(0.8, 0, 0);
                    }
                    break;

                case DRIVE_WITH_FOUNDATION_3:
                    targetTheta = 102;

                    extendGantry(0.5);

                    if (gyroZ < targetTheta) {
                        //Pin foundation fingers against bolt on foundation
                        if (runtime.seconds() < 0.2) {
                            drive(0, -0.2, -0.05);
                        } else if (runtime.seconds() >= 0.2 && runtime.seconds() < 0.35) {
                            drive(0, -0.2, -0.05);
                            sC.setPosition(0.65); //Drop stone
                        } else {
                            changeStep();
                            releaseFoundation();
                            CURRENT_STEP = steps.RETURN_SKYBRIDGE_1;
                        }
                    } else {
                        rotateCounterclockwise(1);
                        runtime.reset();
                    }
                    break;

                case RETURN_SKYBRIDGE_1:
                    targetX = 17;
                    targetY = -30;
                    targetTheta = 90;

                    setIntakePower(1); //Outtake skystone if pickup failed

                    if (isInTolerance(targetX, targetY, targetTheta, 5, 3)) {
                        changeStep();
                        runtimeTwo.reset();
                        //Set step from skystone position
                        if (skystonePos == "left") {
                            CURRENT_STEP = steps.PREP_SKYSTONE_4;
                        } else if (skystonePos == "middle") {
                            CURRENT_STEP = steps.PREP_SKYSTONE_5;
                        } else if (skystonePos == "right") {
                            CURRENT_STEP = steps.PREP_SKYSTONE_6;
                        } else {
                            CURRENT_STEP = steps.PREP_SKYSTONE_4;
                        }
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PREP_SKYSTONE_4:
                    targetX = 23;
                    targetY = -4.2;
                    targetTheta = 40;

                    if (runtime.seconds() < 0.5) {
                        runtimeTwo.reset();
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
                    targetY = -2.4;
                    targetTheta = 40;

                    retractGantry();
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
                    targetX = 20;
                    targetY = -12;
                    targetTheta = 85;

                    retractGantry();

                    if (pose.x < targetX + 3) {
                        changeStep();
                        runtimeTwo.reset();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_2;
                    } else {
                        moveToPoseWaypoint(targetX, targetY, targetTheta, 0.2);
                    }
                    break;

                case PREP_SKYSTONE_5:
                    targetX = 23.5;
                    targetY = -16.2;
                    targetTheta = 50;

                    if (runtime.seconds() < 0.5) {
                        runtimeTwo.reset();
                    } else {
                        retractGantry();
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 5, 3)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_5;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PICK_UP_SKYSTONE_5:
                    targetX = 42;
                    targetY = -10.4;
                    targetTheta = 45;

                    retractGantry();
                    setIntakePower(-1);

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2) || isStoneCollected()) {
                        changeStep();
                        stoneIntakeTime.reset();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_5;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case EXIT_SKYSTONE_5:
                    targetX = 20;
                    targetY = -16;
                    targetTheta = 85;

                    retractGantry();

                    if (pose.x < targetX + 3) {
                        changeStep();
                        runtimeTwo.reset();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_2;
                    } else {
                        moveToPoseWaypoint(targetX, targetY, targetTheta, 0.2);
                    }
                    break;

                case DRIVE_TO_FOUNDATION_2:
                    targetX = 23;
                    targetY = -60;
                    targetTheta = 90;

                    //Extend gantry after clearing skybridge
                    if (pose.y > -40) {
                        runtimeTwo.reset();
                    } else if (pose.y < -40 && pose.y > -50) {
                        runtimeTwo.reset();
                        sC.setPosition(0.45);
                    } else {
                        extendGantry(0.6);
                    }

                    if (pose.y < targetY + 3) {
                        changeStep();
                        CURRENT_STEP = steps.PLACE_SKYSTONE_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PLACE_SKYSTONE_2:
                    extendGantry(0.6);
                    dropStoneWhenDone(2);

                    if (runtimeTwo.seconds() < 2.25) {
                        //Push foundation against wall
                        driveBackward(0.3);
                    } else {
                        changeStep();
                        runtimeTwo.reset();
                        CURRENT_STEP = steps.RETURN_SKYBRIDGE_2;
                    }
                    break;

                case RETURN_SKYBRIDGE_2:
                    targetX = 22;
                    targetY = -28;
                    targetTheta = 90;

                    if (runtime.seconds() < 0.25) {
                        mML.setPower(-0.9);
                        mMR.setPower(-0.9);
                        runtimeTwo.reset();
                    } else {
                        retractGantry();
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 5, 3)) {
                        changeStep();
                        //Set step from skystone position
                        if (skystonePos == "left") {
                            CURRENT_STEP = steps.PREP_STONE_6;
                        } else if (skystonePos == "middle") {
                            CURRENT_STEP = steps.PREP_STONE_4;
                        } else if (skystonePos == "right") {
                            CURRENT_STEP = steps.STOP;
                        } else {
                            CURRENT_STEP = steps.PREP_STONE_6;
                        }
                    } else {
                        //Give time for gantry retraction
                        if (runtime.seconds() < 1.4) {
                            drive(0.25, 0.1, 0);
                        } else {
                            moveToPose(targetX, targetY, targetTheta, 0.4);
                        }
                    }
                    break;

                case PREP_STONE_6:
                    targetX = 27.9;
                    targetY = -21.5;
                    targetTheta = 30;

                    retractGantry();

                    if (isInTolerance(targetX, targetY, targetTheta, 5, 15)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_STONE_6;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.6);
                    }
                    break;

                case PICK_UP_STONE_6:
                    targetX = 42;
                    targetY = -14;
                    targetTheta = 18;

                    setIntakePower(-1);

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2) || isStoneCollected()) {
                        changeStep();
                        stoneIntakeTime.reset();
                        CURRENT_STEP = steps.EXIT_STONE_6;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case EXIT_STONE_6:
                    targetX = 25;
                    targetY = -16;
                    targetTheta = 85;

                    if (pose.x < targetX + 3) {
                        changeStep();
                        runtimeTwo.reset();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_3;
                    } else {
                        moveToPoseWaypoint(targetX, targetY, targetTheta, 0.2);
                    }
                    break;

                case PREP_STONE_4:
                    targetX = 28.6;
                    targetY = -0.4;
                    targetTheta = 37;

                    retractGantry();

                    if (isInTolerance(targetX, targetY, targetTheta, 5, 15)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_STONE_4;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.6);
                    }
                    break;


                case PICK_UP_STONE_4:
                    targetX = 53;
                    targetY = 15;
                    targetTheta = 29;

                    setIntakePower(-1);

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2) || isStoneCollected()) {
                        changeStep();
                        stoneIntakeTime.reset();
                        CURRENT_STEP = steps.EXIT_STONE_4;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case EXIT_STONE_4:
                    targetX = 25;
                    targetY = -6;
                    targetTheta = 85;

                    if (pose.x < targetX + 3) {
                        changeStep();
                        runtimeTwo.reset();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_3;
                    } else {
                        moveToPoseWaypoint(targetX, targetY, targetTheta, 0.2);
                    }
                    break;

                case DRIVE_TO_FOUNDATION_3:
                    targetX = 24;
                    targetY = -70;
                    targetTheta = 90;

                    //Extend gantry after clearing skybridge
                    if (pose.y > -40) {
                        runtimeTwo.reset();
                    } else if (pose.y < -40 && pose.y > -50) {
                        runtimeTwo.reset();
                        sC.setPosition(0.45);
                    } else {
                        extendGantry(0.6);
                    }

                    if (pose.y < targetY + 3) {
                        changeStep();
                        CURRENT_STEP = steps.PLACE_STONE_3;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;

                case PLACE_STONE_3:
                    extendGantry(0.85);
                    dropStoneWhenDone(2);

                    if (runtimeTwo.seconds() < 2.25) {
                        //Push foundation against wall
                        drive(-0.4, 0, 0);
                    } else {
                        changeStep();
                        runtimeTwo.reset();
                        CURRENT_STEP = steps.PARK;
                    }
                    break;

                case PARK:
                    targetX = 22;
                    targetY = -35;
                    targetTheta = 90;

                    retractGantry();

                    if (isInTolerance(targetX, targetY, targetTheta, 5, 3)) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        //Give time for gantry retraction
                        if (runtime.seconds() < 1.4) {
                            drive(0.25, 0.1, 0);
                        } else {
                            moveToPose(targetX, targetY, targetTheta, 0.4);
                        }
                    }
                    break;
                case STOP:
                    //Stop powering motors
                    mML.setPower(0);
                    mMR.setPower(0);
                    setIntakePower(0);
                    stopDriveTrain();
                    break;
            }
        }
        shutdown();
    }
}