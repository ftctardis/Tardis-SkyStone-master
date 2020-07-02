package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvCameraRotation;

//Name stands for Place 3 Stones (first two are skystones), Reposition, Navigate
@Autonomous(name = "Blue_Depot_49_P3SRN", group = "Autonomous")

public class Blue_Depot_49_P3SRN extends AutoBaseTB1 {

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
            telemetry.addData("Pos x", String.format("%.1f", pose.x));
            telemetry.addData("Pos y", String.format("%.1f", pose.y));
            telemetry.addData("GyroZ", String.format("%.1f", gyroZ));
            telemetry.addData("Stone collected", isStoneCollected());
            telemetry.addData("Skystone 2 X", String.format("%.1f", skystoneTwoXPos));
            telemetry.update();

            switch (CURRENT_STEP) {

                //Scan stones 3, 4, and 5
                case WEBCAM_CHECK:
                    if (skystonePos == "left") {
                        CURRENT_STEP = steps.PREP_SKYSTONE_5;
                    } else if (skystonePos == "middle") {
                        CURRENT_STEP = steps.PREP_SKYSTONE_1;
                    } else if (skystonePos == "right") {
                        CURRENT_STEP = steps.PREP_SKYSTONE_6;
                    } else {
                        CURRENT_STEP = steps.PREP_SKYSTONE_6;
                    }
                    break;


                case PREP_SKYSTONE_1:
                    targetX = 25.2;
                    targetY = -20.5;
                    targetTheta = -28.7;

                    //Moves capstone out of way
                    if (runtime.seconds() > 1) {
                        sCS.setPosition(0.05);
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_1;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case PICK_UP_SKYSTONE_1:
                    targetX = 39.8;
                    targetY = -22.4;
                    targetTheta = -42.8;

                    //Intake and open gripper
                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || isStoneCollected()){//runtime.seconds() > 3) {
                        changeStep();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_1;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case EXIT_SKYSTONE_1:
                    targetX = 28.7;
                    targetY = -10.4;
                    targetTheta = -60;

                    //Stop powering mast
                    mML.setPower(0);
                    mMR.setPower(0);

                    if (pose.x < targetX + 3) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        moveToPoseWaypoint(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case PREP_SKYSTONE_5:
                    targetX = 29.7;
                    targetY = 18.4;
                    targetTheta = -34.7;

                    //Moves capstone out of way
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

                    //Intake and open gripper
                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || isStoneCollected()){//runtime.seconds() > 2.5) {
                        changeStep();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_5;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case EXIT_SKYSTONE_5:
                    targetX = 28.7;
                    targetY = 7.7;
                    targetTheta = -30.6;

                    if (pose.x < targetX + 3) {
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

                    //Moves capstone out of way
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

                    //Intake and open gripper
                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || isStoneCollected()){//runtime.seconds() > 2.5) {
                        changeStep();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_6;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case EXIT_SKYSTONE_6:
                    targetX = 28.7;
                    targetY = 16.7;
                    targetTheta = -30.6;

                    if (pose.x < targetX + 3) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case DRIVE_TO_SKYBRIDGE:
                    targetX = 26.3;
                    targetY = 39.4;
                    targetTheta = -90;

                    if (pose.y > targetY - 4) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.2);
                    }
                    break;


                case DRIVE_TO_FOUNDATION:
                    targetX = 18.4;
                    targetY = 91.6;
                    targetTheta = -180;

                    //Grab stone
                    if (runtime.seconds() > 1) {
                        setIntakePower(0);
                        sC.setPosition(0.45);
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 2.5, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.GRAB_FOUNDATION;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case GRAB_FOUNDATION:
                    targetX = 23.8;

                    if (pose.x > targetX) {
                        changeStep();
                        runtimeTwo.reset();
                        CURRENT_STEP = steps.DRIVE_WITH_FOUNDATION;
                    } else {
                        driveForwardGyro(-0.5, -180, 2);
                    }
                    break;


                case DRIVE_WITH_FOUNDATION:
                    //Prep for placing stone
                    holdFoundation();
                    extendGantry(0.75);
                    dropStoneWhenDone(3.25);
                    setIntakePower(1); //Output stone if caught in intake

                    //Grab foundation, rotate slightly, then pull foundation back
                    if (runtime.seconds() < 0.5) {
                        stopDriveTrain();
                    } else if (runtime.seconds() >= 0.5 && runtime.seconds() < 1) {
                        drive(0, -0.6, -1);
                    } else {
                        if (pose.x > 17) {
                            driveForwardGyro(1, -90, 2);
                        } else {
                            changeStep();
                            CURRENT_STEP = steps.DRIVE_WITH_FOUNDATION_2;
                        }
                    }
                    break;


                case DRIVE_WITH_FOUNDATION_2:
                    //Continue prep for placing stones
                    extendGantry(0.75);
                    dropStoneWhenDone(3.25);

                    //If stone is placed, run mast up
                    if (runtimeTwo.seconds() > 3.5 && runtimeTwo.seconds() < 4) {
                        mML.setPower(-0.6);
                        mMR.setPower(-0.6);
                    }

                    //Rotate the foundation
                    if (gyroZ < -105) {
                        rotateCounterclockwise(1);
                    } else {
                        if (runtimeTwo.seconds() < 4) {
                            stopDriveTrain();
                        } else {
                            changeStep();
                            skystoneTwoXPos = pose.x; //Sets skystone 2 X for later
                            CURRENT_STEP = steps.DRIVE_BACK;
                        }
                    }
                    break;


                case DRIVE_BACK:
                    targetX = 19.9;
                    targetY = 36.5;
                    targetTheta = -90;

                    //Let go of foundaiton
                    setIntakePower(0);
                    releaseFoundation();

                    //Raise mast to clear stone
                    if (runtimeTwo.seconds() < 4.25) {
                        mML.setPower(-0.6);
                        mMR.setPower(-0.6);
                        stopDriveTrain();
                    } else {

                        //Hold mast at height, then retract
                        if (runtimeTwo.seconds() >= 4.25 && runtimeTwo.seconds() < 4.75) {
                            mML.setPower(-0.2);
                            mMR.setPower(-0.2);
                        } else {
                            mML.setPower(0.2);
                            mMR.setPower(0.2);
                        }

                        if (pose.y < targetY + 3) {
                            changeStep();

                            //Set stone based on skystonePos from init
                            if (skystonePos == "left") {
                                CURRENT_STEP = steps.PREP_SKYSTONE_2;
                            } else if (skystonePos == "middle") {
                                CURRENT_STEP = steps.PREP_SKYSTONE_4;
                            } else if (skystonePos == "right") {
                                CURRENT_STEP = steps.PREP_SKYSTONE_3;
                            } else {
                                CURRENT_STEP = steps.PREP_SKYSTONE_3;
                            }
                        } else {
                            moveToPose(targetX, targetY, targetTheta, 0.5);
                        }
                    }
                    break;


                case PREP_SKYSTONE_4:
                    targetX = 22.8;
                    targetY = 6.5;
                    targetTheta = -37.9;

                    //Clear skybridge then retract gantry
                    if (runtime.seconds() < 0.75) {
                        runtimeTwo.reset();
                    } else {
                        retractGantry();
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_4;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case PICK_UP_SKYSTONE_4:
                    targetX = 39.5;
                    targetY = -0.1;
                    targetTheta = -27;

                    //Continue retracting gantry, run intake, open gripper
                    retractGantry();
                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || isStoneCollected()){//runtime.seconds() > 2.5) {
                        changeStep();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_4;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case EXIT_SKYSTONE_4:
                    targetX = 19.8;
                    targetY = 7.4;
                    targetTheta = -60;

                    if (pose.x < targetX + 3) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;//DRIVE_TO_SKYBRIDGE_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case PREP_SKYSTONE_2:
                    targetX = 19.2;
                    targetY = -8.2;
                    targetTheta = -40.7;

                    //Clear skybridge then retract gantry
                    if (runtime.seconds() < 0.75) {
                        runtimeTwo.reset();
                    } else {
                        retractGantry();
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.8);
                    }
                    break;


                case PICK_UP_SKYSTONE_2:
                    targetX = 35.8;
                    targetY = -12.8;
                    targetTheta = -44.8;

                    //Continue retracting gantry, run intake, open gripper
                    retractGantry();
                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || isStoneCollected()){//runtime.seconds() > 2.5) {
                        changeStep();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case EXIT_SKYSTONE_2:
                    targetX = 28.7;
                    targetY = -3.4;
                    targetTheta = -60;

                    //Stop powering mast
                    mML.setPower(0);
                    mMR.setPower(0);

                    if (pose.x < targetX + 3) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case PREP_SKYSTONE_3:
                    targetX = 19.2;
                    targetY = -1.2;
                    targetTheta = -40.7;

                    //Clear skybridge then retract gantry
                    if (runtime.seconds() < 0.75) {
                        runtimeTwo.reset();
                    } else {
                        retractGantry();
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_3;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.8); //0.4
                    }
                    break;


                case PICK_UP_SKYSTONE_3:
                    targetX = 37.3;
                    targetY = -6.4;
                    targetTheta = -44.8;

                    //Continue retracting gantry, run intake, open gripper
                    retractGantry();
                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || isStoneCollected()){//runtime.seconds() > 2.5) {
                        changeStep();
                        CURRENT_STEP = steps.EXIT_SKYSTONE_3;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case EXIT_SKYSTONE_3:
                    targetX = 28.7;
                    targetY = 3.4;
                    targetTheta = -60;

                    //Stop powering mast
                    mML.setPower(0);
                    mMR.setPower(0);

                    if (pose.x < targetX + 3) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case DRIVE_TO_SKYBRIDGE_2:
                    targetX = 19.1;
                    targetY = 39.4;
                    targetTheta = -90;

                    holdFoundation(); //Push foundation hooks down to line up 2nd stone

                    if (pose.y > targetY - 4) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_2;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.3);
                    }
                    break;


                case DRIVE_TO_FOUNDATION_2:
                    targetX = skystoneTwoXPos - 1.25; //Set targetX based on position after moving foundation
                    targetY = 75.3;
                    targetTheta = -90;

                    //Grab stone
                    setIntakePower(0);
                    sC.setPosition(0.45);

                    //Clear skybridge, prep for placing stone
                    if (runtime.seconds() < 0.5) {
                        runtimeTwo.reset();
                    } else {
                        extendGantry(1);
                        dropStoneWhenDone(3);
                    }

                    //Be inside 0.25" tolerance for at least a half second to account for drift
                    if (isInTolerance(targetX, targetY, targetTheta, 0.25, 2) && runtime.seconds() > 0.5) {
                        changeStep();
                        CURRENT_STEP = steps.PLACE_SKYSTONE_2;
                    } else {
                        if (isInTolerance(targetX, targetY, targetTheta, 0.25, 2)) {
                            stopDriveTrain();
                        } else {
                            moveToPose(targetX, targetY, targetTheta, 0.4);
                            runtime.reset();
                        }
                    }
                    break;


                case PLACE_SKYSTONE_2:
                    //Continue prep for placing stone
                    extendGantry(1);
                    dropStoneWhenDone(3);

                    if (runtime.seconds() > 1 && runtimeTwo.seconds() > 3.25) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_BACK_2;
                    } else {
                        driveForwardGyro(-0.3, -90, 2);
                    }
                    break;


                case DRIVE_BACK_2:
                    targetX = 25.9;
                    targetY = 39.4;
                    targetTheta = -90;

                    //Let go of foundaiton
                    setIntakePower(0);
                    releaseFoundation();

                    //Raise mast to clear stone
                    if (runtimeTwo.seconds() < 3.75) {
                        mML.setPower(-0.6);
                        mMR.setPower(-0.6);
                        stopDriveTrain();
                    } else {

                        //Hold mast at height, then retract
                        if (runtimeTwo.seconds() >= 4.25 && runtimeTwo.seconds() < 4.75) {
                            mML.setPower(-0.2);
                            mMR.setPower(-0.2);
                        } else {
                            mML.setPower(0.2);
                            mMR.setPower(0.2);
                        }

                        if (pose.y < targetY + 3) {
                            changeStep();

                            //Set stone based on skystonePos from init
                            if (skystonePos == "left") {
                                CURRENT_STEP = steps.STOP;
                            } else if (skystonePos == "middle") {
                                CURRENT_STEP = steps.PREP_STONE_6;
                            } else if (skystonePos == "right") {
                                CURRENT_STEP = steps.STOP;
                            } else {
                                CURRENT_STEP = steps.STOP;
                            }
                        } else {
                            moveToPose(targetX, targetY, targetTheta, 0.5);
                        }
                    }
                    break;


                case PREP_STONE_6:
                    targetX = 23.9;
                    targetY = 21.8;
                    targetTheta = -57.3;

                    //Moves capstone out of way
                    if (runtime.seconds() > 1) {
                        sCS.setPosition(0.05);
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 3, 10)) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_STONE_6;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case PICK_UP_STONE_6:
                    targetX = 36.7;
                    targetY = 13.7;
                    targetTheta = -34.1;

                    //Intake and open gripper
                    setIntakePower(-1);
                    sC.setPosition(0.7);

                    if (isInTolerance(targetX, targetY, targetTheta, 1.5, 2) || isStoneCollected()){//runtime.seconds() > 2.5) {
                        changeStep();
                        CURRENT_STEP = steps.EXIT_STONE_6;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case EXIT_STONE_6:
                    targetX = 22.2;
                    targetY = 19.6;
                    targetTheta = -74.2;

                    if (pose.x < targetX + 3) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE_3;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.4);
                    }
                    break;


                case DRIVE_TO_SKYBRIDGE_3:
                    targetX = 26.3;
                    targetY = 39.4;
                    targetTheta = -90;

                    if (pose.y > targetY - 4) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_3;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.2);
                    }
                    break;


                case DRIVE_TO_FOUNDATION_3:
                    targetX = skystoneTwoXPos - 1.25; //Set targetX based on position after moving foundation
                    targetY = 75.3;
                    targetTheta = -90;

                    //Grab stone
                    setIntakePower(0);
                    sC.setPosition(0.45);

                    //Clear skybridge, prep for placing stone
                    if (runtime.seconds() < 0.5) {
                        runtimeTwo.reset();
                    } else {
                        extendGantry(1);
                        dropStoneWhenDone(3);
                    }

                    //Be inside 0.25" tolerance for at least a half second to account for drift
                    if (isInTolerance(targetX, targetY, targetTheta, 0.25, 2) && runtime.seconds() > 0.5) {
                        changeStep();
                        CURRENT_STEP = steps.PLACE_STONE_3;
                    } else {
                        if (isInTolerance(targetX, targetY, targetTheta, 0.25, 2)) {
                            stopDriveTrain();
                        } else {
                            moveToPose(targetX, targetY, targetTheta, 0.4);
                            runtime.reset();
                        }
                    }
                    break;


                case PLACE_STONE_3:
                    //Continue prep for placing stone
                    extendGantry(1);
                    dropStoneWhenDone(3);

                    if (runtime.seconds() > 1 && runtimeTwo.seconds() > 3.25) {
                        changeStep();
                        runtimeTwo.reset();
                        CURRENT_STEP = steps.PREP_PARK;
                    } else {
                        driveForwardGyro(-0.3, -90, 2);
                    }
                    break;


                case PREP_PARK:
                    targetX = 20.1;
                    targetY = 60.4;
                    targetTheta = -90;

                    //After placement, lift mast while driving into wall
                    if (runtimeTwo.seconds() < 0.5) {
                        driveBackward(0.4);
                        mML.setPower(-0.9);
                        mMR.setPower(-0.9);
                    } else if (runtimeTwo.seconds() >= 0.75 && runtimeTwo.seconds() < 1.5) {
                        //Retract gantry
                        sG.setPosition(1);
                        mML.setPower(-0.2);
                        mMR.setPower(-0.2);
                    } else {
                        //Retract mast
                        mML.setPower(0.2);
                        mMR.setPower(0.2);
                    }

                    if (runtimeTwo.seconds() > 0.5) {
                        if (isInTolerance(targetX, targetY, targetTheta, 3, 2) && runtimeTwo.seconds() > 1) {
                            changeStep();
                            CURRENT_STEP = steps.PARK;
                        } else {
                            moveToPose(targetX, targetY, targetTheta, 0.8);
                        }
                    }
                    break;


                case PARK:
                    targetX = 19.8;
                    targetY = 36.4;
                    targetTheta = -90;

                    releaseFoundation();

                    //Continue retracting gantry and mast
                    if (runtimeTwo.seconds() < 1.5) {
                        mML.setPower(-0.2);
                        mMR.setPower(-0.2);
                    } else {
                        mML.setPower(0.2);
                        mMR.setPower(0.2);
                    }

                    if (isInTolerance(targetX, targetY, targetTheta, 2, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        moveToPose(targetX, targetY, targetTheta, 0.8);
                    }
                    break;


                case STOP:
                    //Stop powering motors
                    mML.setPower(0);
                    mMR.setPower(0);
                    stopDriveTrain();
                    break;
            }
        }
        shutdown();
    }
}