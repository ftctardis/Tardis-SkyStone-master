package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red_Depot_25_TB1_D2SN", group = "Autonomous")
@Disabled

public class Red_Depot_25_TB1_D2SN extends AutoBaseTB1 {

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

                case WEBCAM_CHECK:
                    if (skystonePos == "left") {
                        CURRENT_STEP = steps.PREP_SKYSTONE_4;
                    } else if (skystonePos == "middle") {
                        CURRENT_STEP = steps.PREP_SKYSTONE_5;
                    } else if (skystonePos == "right") {
                        CURRENT_STEP = steps.PREP_SKYSTONE_6;
                    } else {
                        CURRENT_STEP = steps.PREP_SKYSTONE_5;
                    }
                    break;

                case PREP_SKYSTONE_4:
                    targetX = 25;
                    targetY = -7;

                    if (isInToleranceOLD(targetX, targetY, 3)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_SKYSTONE_4;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case ROTATE_SKYSTONE_4:
                    if (gyroZ > 88 && gyroZ < 92) {
                        changeStep();
                        CURRENT_STEP = steps.ALIGN_SKYSTONE;
                    } else {
                        rotateSigmoid(90);
                    }
                    break;

                case ALIGN_SKYSTONE:
                    targetX = 39;
                    setIntakePower(0);

                    if (pose.x > targetX) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_4;
                    } else {
                        driveRightGyro(0.5, 90, 2);
                    }
                    break;

                case PICK_UP_SKYSTONE_4:
                    setIntakePower(-1);

                    if (runtime.seconds() > 0.5) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_FOUNDATION;
                    } else {
                        driveForwardGyro(0.5, 90, 2);
                    }
                    break;

                case PREP_SKYSTONE_5:
                    targetX = 23;
                    targetY = 6;

                    if (isInToleranceOLD(targetX, targetY, 2.5)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_SKYSTONE_5;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case ROTATE_SKYSTONE_5:
                    if (gyroZ < -35 && gyroZ > -39) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_5;
                    } else {
                        rotateSigmoid(-40);
                    }
                    break;

                case PICK_UP_SKYSTONE_5:
                    setIntakePower(-1);

                    if (runtime.seconds() > 1.2) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_AFTER_SKYSTONE_6;
                    } else {
                        driveForwardGyro(0.5, -35, 2);
                    }
                    break;

                case PREP_SKYSTONE_6:
                    targetX = 23;
                    targetY = -3;

                    if (isInToleranceOLD(targetX, targetY, 2)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_SKYSTONE_6;
                    } else {
                        moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case ROTATE_SKYSTONE_6:
                    if (gyroZ < -45 && gyroZ > -49) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_6;
                    } else {
                        rotateSigmoid(-50);
                    }
                    break;

                case PICK_UP_SKYSTONE_6:
                    targetX = 45;
                    targetY = -11;
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

                    if (gyroZ > 85) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_FOUNDATION;
                    } else {
                        rotateSigmoid(90);
                    }
                    break;

                case ROTATE_TOWARDS_FOUNDATION:
                    setIntakePower(0);
                    targetX = 26;

                    if (pose.x < targetX) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE;
                    } else {
                        driveRightGyro(-0.5, 90, 2);
                    }
                    break;

                case DRIVE_TO_SKYBRIDGE:
                    targetX = 26;
                    targetY = -26;

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION;
                    } else {
                        moveToPoseOLD(targetX, targetY, -1, true, 1, false); //Not strafe
                    }
                    break;

                case DRIVE_TO_FOUNDATION:
                    targetX = 26;
                    targetY = -60;

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_DEPOSIT;
                    } else {
                        moveToPoseOLD(targetX, targetY, -1, true, 2, true); //Not strafe
                    }
                    break;

                case ROTATE_DEPOSIT:
                    if (gyroZ < 5) {
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
                    if (gyroZ > 85) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_BACK;
                    } else {
                        rotateSigmoid(90);
                    }
                    break;

                case DRIVE_BACK:
                    targetX = 26;
                    targetY = -26;

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        if (skystonePos == "left") {
                            CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_1;
                        } else if (skystonePos == "middle") {
                            CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_2;
                        } else if (skystonePos == "right") {
                            CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_3;
                        } else {
                            CURRENT_STEP = steps.DRIVE_TO_SKYSTONE_2;
                        }
                    } else {
                        moveToPoseOLD(targetX, targetY, 1, true, 2, false); //Not strafe
                    }
                    break;

                case DRIVE_TO_SKYSTONE_1:
                    targetX = 26;
                    targetY = 15;

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_SKYSTONE_1;
                    } else {
                        moveToPoseOLD(targetX, targetY, 1, true, 1.5, true);
                        //moveToPoseStrafeOLD(targetX, targetY, 1, true, 2.5, true);
                    }
                    break;

                case ROTATE_SKYSTONE_1:
                    if (gyroZ > 88 && gyroZ < 92) {
                        changeStep();
                        CURRENT_STEP = steps.PREP_SKYSTONE_1;
                    } else {
                        rotateSigmoid(90);
                    }
                    break;

                case PREP_SKYSTONE_1:
                    targetX = 38;

                    if (pose.x > targetX) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_1;
                    } else {
                        driveRightGyro(0.5, 90, 2);
                    }
                    break;

                case PICK_UP_SKYSTONE_1:
                    targetY = 21.5;
                    setIntakePower(-1);

                    if (runtime.seconds() > 0.8) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_FOUNDATION_2;
                    } else {
                        driveForwardGyro(0.4, 90, 2);
                    }
                    break;

                case DRIVE_TO_SKYSTONE_2:
                    targetX = 27;
                    targetY = 6; // 10

                    if (isInToleranceOLD(targetX, targetY, 5)) { // is in tolerance
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_SKYSTONE_2;
                    } else {
                        moveToPoseOLD(targetX, targetY, 1, true, 1.5, true);
                    }
                    break;

                case ROTATE_SKYSTONE_2:
                    if (gyroZ > 88 && gyroZ < 92) {
                        changeStep();
                        CURRENT_STEP = steps.PREP_SKYSTONE_2;
                    } else {
                        rotateSigmoid(90);
                    }
                    break;

                case PREP_SKYSTONE_2:
                    targetX = 38;

                    if (pose.x > targetX) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_2;
                    } else {
                        driveRightGyro(0.5, 90, 1);
                    }
                    break;

                case PICK_UP_SKYSTONE_2:
                    targetY = 13;
                    setIntakePower(-1);

                    if (runtime.seconds() > 0.8) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_FOUNDATION_2;
                    } else {
                        driveForwardGyro(0.4, 90, 2);
                    }
                    break;

                case DRIVE_TO_SKYSTONE_3:
                    targetX = 26;
                    targetY = 0;

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_SKYSTONE_3;
                    } else {
                        moveToPoseOLD(targetX, targetY, 1, true, 1.5, true);
                    }
                    break;

                case ROTATE_SKYSTONE_3:
                    if (gyroZ > 88 && gyroZ < 92) {
                        changeStep();
                        CURRENT_STEP = steps.PREP_SKYSTONE_3;
                    } else {
                        rotateSigmoid(90);
                    }
                    break;

                case PREP_SKYSTONE_3:
                    targetX = 38;

                    if (pose.x > targetX) {
                        changeStep();
                        CURRENT_STEP = steps.PICK_UP_SKYSTONE_3;
                    } else {
                        driveRightGyro(0.5, 90, 2);
                    }
                    break;

                case PICK_UP_SKYSTONE_3:
                    targetY = 5.5;
                    setIntakePower(-1);

                    if (runtime.seconds() > 0.8) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TOWARDS_FOUNDATION_2;
                    } else {
                        driveForwardGyro(0.4, 90, 2);
                    }
                    break;

                case ROTATE_TOWARDS_FOUNDATION_2:
                    targetX = 30;
                    setIntakePower(0);

                    if (pose.x < targetX) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_SKYBRIDGE_2;
                    } else {
                        driveRightGyro(-0.5, 90, 2);
                    }
                    break;

                case DRIVE_TO_SKYBRIDGE_2:
                    targetX = 26;
                    targetY = -26;

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_FOUNDATION_2;
                    } else {
                        moveToPoseOLD(targetX, targetY, -1, true, 1.5, false); //Not strafe
                    }
                    break;

                case DRIVE_TO_FOUNDATION_2:
                    targetX = 26;
                    targetY = -60;

                    if (isInToleranceOLD(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_DEPOSIT_2;
                    } else {
                        moveToPoseOLD(targetX, targetY, -1, true, 1.5, true); //Not strafe
                    }
                    break;

                case ROTATE_DEPOSIT_2:
                    if (gyroZ < 5) {
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
                    if (gyroZ > 88) {
                        changeStep();
                        CURRENT_STEP = steps.PARK;
                    } else {
                        rotateSigmoid(90);
                        sP.setPosition(0.4);
                    }
                    break;

                case PARK:
                    if (runtime.seconds() > 1.2) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        driveForwardGyro(0.5, 90, 2);
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