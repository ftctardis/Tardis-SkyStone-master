package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue_Depot_25_TB1_D2SN", group = "Autonomous")
@Disabled

public class Blue_Depot_25_TB1_D2SN extends AutoBaseTB1 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.WEBCAM_CHECK;

        //Init functions
        defineComponents();
        sCS.setPosition(0.58);

        double targetX = 0;
        double targetY = 0;

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
            updatePoseStrafe();
            gyroUpdate();

            telemetry.addData("Current Step", CURRENT_STEP);
            telemetry.addData("Pos x", pose.x);
            telemetry.addData("Pos y", pose.y);
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
                        CURRENT_STEP = steps.PREP_SKYSTONE_4;
                    }
                    break;

                case PREP_SKYSTONE_4:
                    targetX = 37;
                    targetY = 14;

                    if (isInToleranceOLD(targetX, targetY, 3)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_SKYSTONE_4;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case ROTATE_SKYSTONE_4:
                    if (gyroZ < -85) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_4;
                    } else {
                        rotateSigmoid(-90);
                    }
                    break;

                case PICK_UP_SKYSTONE_4:
                    targetX = 38;
                    targetY = 6;
                    setIntakePower(-1);

                    if (isInToleranceOLD(targetX, targetY, 1.5)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_FOUNDATION;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case PREP_SKYSTONE_5:
                    targetX = 23.5;
                    targetY = 14;

                    if (isInToleranceOLD(targetX, targetY, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_SKYSTONE_5;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case ROTATE_SKYSTONE_5:
                    if (gyroZ < -40) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_5;
                    } else {
                        rotateSigmoid(-45);
                    }
                    break;

                case PICK_UP_SKYSTONE_5:
                    targetX = 38;
                    targetY = 13;
                    setIntakePower(-1);

                    if (isInToleranceOLD(targetX, targetY, 1.5)) {
                        if (gyroZ < -55) {
                            changeStep();
                            CURRENT_STEP = steps.ROTATE_TOWARDS_FOUNDATION;
                        } else {
                            rotateSigmoid(-60);
                        }
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case PREP_SKYSTONE_6:
                    targetX = 23;
                    targetY = 2;

                    if (isInToleranceOLD(targetX, targetY, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_SKYSTONE_6;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case ROTATE_SKYSTONE_6:
                    if (gyroZ > 40) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_6;
                    } else {
                        rotateSigmoid(45);
                    }
                    break;

                case PICK_UP_SKYSTONE_6:
                    targetX = 45;
                    targetY = 11;
                    setIntakePower(-1);

                    if (isInToleranceOLD(targetX, targetY, 1.5)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_AFTER_SKYSTONE_6;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case ROTATE_AFTER_SKYSTONE_6:
                    setIntakePower(0);

                    if (gyroZ < -85) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_FOUNDATION;
                    } else {
                        rotateSigmoid(-90);
                    }
                    break;

                case ROTATE_TOWARDS_FOUNDATION:
                    targetX = 27;
                    setIntakePower(0);

                    if (pose.x < targetX) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        driveRightGyro(0.5, -90, 2);
                    }
                    break;

                case DRIVE_TO_SKYBRIDGE:
                    targetX = 25;
                    targetY = 26;

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION;
                    } else {
                        moveToPoseOLD(targetX, targetY, -1, true, 2.5, false); //Not strafe
                    }
                    break;

                case DRIVE_TO_FOUNDATION:
                    targetX = 25;
                    targetY = 60;

                    if (isInToleranceOLD(targetX, targetY, 2.5)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_DEPOSIT;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, -1, true, 2.5, true);
                    }
                    break;

                case ROTATE_DEPOSIT:
                    if (gyroZ > -5) {
                        changeStep();
                        CURRENT_STEP = steps.DEPOSIT;
                    } else {
                        rotateSigmoid(0);
                    }
                    break;

                case DEPOSIT:
                    if (runtime.seconds() > 1) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_BACK;
                    } else {
                        setIntakePower(1);
                    }
                    break;

                case ROTATE_BACK:
                    setIntakePower(0);
                    if (gyroZ < -85) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_BACK;
                    } else {
                        rotateSigmoid(-90);
                    }
                    break;

                case DRIVE_BACK:
                    targetX = 25;
                    targetY = 26;

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        if (skystonePos == "left") {
                            CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_2;
                        } else if (skystonePos == "middle") {
                            CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_1;
                        } else if (skystonePos == "right") {
                            CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_3;
                        } else {
                            CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_1;
                        }
                    } else {
                        moveToPoseOLD(targetX, targetY, 1, true, 2.5, false); //Not strafe
                    }
                    break;


                case DRIVE_TO_SKYSTONE_1:
                    targetX = 20;
                    targetY = -1;

                    if (isInToleranceOLD(targetX, targetY, 2.5)) {
                        changeStep();
                        CURRENT_STEP = steps.PREP_SKYSTONE_1;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case PREP_SKYSTONE_1:
                    targetX = 38;

                    if (pose.x > targetX) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_1;
                    } else {
                        driveRightGyro(-0.5, -90, 2);
                    }
                    break;


                case PICK_UP_SKYSTONE_1:
                    targetY = -18;
                    setIntakePower(-1);

                    if (pose.y < targetY) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_FOUNDATION_2;
                    } else {
                        driveForwardGyro(0.4, -90, 2);
                    }
                    break;

                case DRIVE_TO_SKYSTONE_2:
                    targetX = 25;
                    targetY = 2;

                    if (isInToleranceOLD(targetX, targetY, 2.5)) {
                        changeStep();
                        CURRENT_STEP = steps.PREP_SKYSTONE_2;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case PREP_SKYSTONE_2:
                    targetX = 38;

                    if (pose.x > targetX) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_2;
                    } else {
                        driveRightGyro(-0.5, -90, 2);
                    }
                    break;


                case PICK_UP_SKYSTONE_2:
                    targetY = -12;
                    setIntakePower(-1);

                    if (pose.y < targetY) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_FOUNDATION_2;
                    } else {
                        driveForwardGyro(0.4, -90, 2);
                    }
                    break;

                case DRIVE_TO_SKYSTONE_3:
                    targetX = 25;
                    targetY = 11;

                    if (isInToleranceOLD(targetX, targetY, 2.5)) {
                        changeStep();
                        CURRENT_STEP = steps.PREP_SKYSTONE_3;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case PREP_SKYSTONE_3:
                    targetX = 38;

                    if (pose.x > targetX) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_3;
                    } else {
                        driveRightGyro(-0.5, -90, 2);
                    }
                    break;


                case PICK_UP_SKYSTONE_3:
                    targetY = -4;
                    setIntakePower(-1);

                    if (pose.y < targetY) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_FOUNDATION_2;
                    } else {
                        driveForwardGyro(0.4, -90, 2);
                    }
                    break;

                case ROTATE_TOWARDS_FOUNDATION_2:
                    targetX = 27;
                    setIntakePower(0);

                    if (pose.x < targetX) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE_2;
                    } else {
                        driveRightGyro(0.5, -90, 2);
                    }
                    break;


                case DRIVE_TO_SKYBRIDGE_2:
                    targetX = 25;
                    targetY = 26;

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_2;
                    } else {
                        moveToPoseOLD(targetX, targetY, -1, true, 3.25, false); //Not strafe
                    }
                    break;

                case DRIVE_TO_FOUNDATION_2:
                    targetX = 25;
                    targetY = 60;

                    if (isInToleranceOLD(targetX, targetY, 2.5)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_DEPOSIT_2;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, -1, true, 2.5, true);
                    }
                    break;

                case ROTATE_DEPOSIT_2:
                    if (gyroZ > -5) {
                        changeStep();
                        CURRENT_STEP = steps.DEPOSIT_2;
                    } else {
                        rotateSigmoid(0);
                    }
                    break;


                case DEPOSIT_2:
                    if (runtime.seconds() > 1) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_PARK;
                    } else {
                        setIntakePower(1);
                    }
                    break;

                case ROTATE_PARK:
                    setIntakePower(0);
                    if (gyroZ < -85) {
                        changeStep();
                        CURRENT_STEP = steps.PARK;
                    } else {
                        rotateSigmoid(-90);
                    }
                    break;

                case PARK:
                    targetX = 25;
                    targetY = 41;

                    if (isInToleranceOLD(targetX, targetY, 2.5)) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
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