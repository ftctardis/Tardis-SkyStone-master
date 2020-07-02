//TestBed Rover Ruckus
package org.firstinspires.ftc.teamcode.skystone.TB1;

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.skystone.TB0.BaseClassTB0;

@TeleOp(name = "TeleOp_TB1_TEST", group = "TeleOp")
@Disabled

public class TeleOp_TB1_TEST extends BaseClassTB1 {

    @Override
    public void runOpMode() {

        defineComponents();
        double targetPosX = 0;
        double targetPosY = 0;
        float targetGyro = 0;
        double gyroLock = 0;
        double strafeAssist = 0;
        boolean isAutoMove = false;
        waitForStart();

        while (opModeIsActive()) {

            gyroUpdate();
            updatePose();

            double leftX = -gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y;
            double rightX = -gamepad1.right_stick_x;


            /*
            if(rightX != 0) {
                runtime.reset();
            }

            if(gamepad1.right_bumper) {
                isAutoMove = true;
                targetPosX = 0;
                targetPosY = 5500;
                targetGyro = gyroZ;
            }

            if(leftX != 0 || leftY != 0) {
                isAutoMove = false;
            }

            if(gyroZ > gyroLock + 2) {
                strafeAssist = (gyroZ - gyroLock) / 25;
            } else if(gyroZ < gyroLock - 2) {
                strafeAssist = (gyroZ - gyroLock) / 25;
            } else {
                strafeAssist = 0;
            }

            if(isAutoMove) {
                double distanceY = targetPosY - posY;
                double distanceX = targetPosX - posX;
                double distanceGyro = targetGyro - gyroZ;

                leftY = (2 / (1 + Math.pow(Math.E, -0.0009 * distanceY))) - 1;
                leftX = (2 / (1 + Math.pow(Math.E, -0.0012 * distanceX))) - 1;
                rightX = (2 / (1 + Math.pow(Math.E, -0.02 * distanceGyro))) - 1;

                if(Math.abs(distanceY) < 100) {
                    leftY = 0;
                }

                if(Math.abs(distanceX) < 100) {
                    leftX = 0;
                }

                if(Math.abs(distanceGyro) < 5) {
                    rightX = 0;
                }

                if(Math.abs(distanceX) < 100 && Math.abs(distanceY) < 100) {
                    isAutoMove = false;
                }
                strafeAssist = 0;
            }

            if(runtime.seconds() < 1) {
                gyroLock = gyroZ;
            }
            */
            double mFLPower = (leftY + -Math.cos(Math.toRadians(2 * gyroZ)) * leftX) * Math.sin(Math.toRadians(gyroZ) + (Math.PI / 4)) - rightX + strafeAssist;
            double mFRPower = (leftY - -Math.cos(Math.toRadians(2 * gyroZ)) * leftX) * Math.cos(Math.toRadians(gyroZ) + (Math.PI / 4)) + rightX - strafeAssist;
            double mBLPower = (leftY - -Math.cos(Math.toRadians(2 * gyroZ)) * leftX) * Math.cos(Math.toRadians(gyroZ) + (Math.PI / 4)) - rightX + strafeAssist;
            double mBRPower = (leftY + -Math.cos(Math.toRadians(2 * gyroZ)) * leftX) * Math.sin(Math.toRadians(gyroZ) + (Math.PI / 4)) + rightX - strafeAssist;

            telemetry.addData("mFL", mFLPower);
            telemetry.addData("mFR", mFRPower);
            telemetry.addData("mBL", mBLPower);
            telemetry.addData("mBR", mBRPower);
            telemetry.update();

            mFL.setPower(mFLPower);
            mFR.setPower(mFRPower);
            mBL.setPower(mBLPower);
            mBR.setPower(mBRPower);

            if(Math.abs(leftY) > 0.2) {
                rightX *= 0.9;
            }
            /*
            mFL.setPower((leftY + leftX) * Math.cos(Math.toRadians(gyroZ) + (Math.PI / 4)) - rightX + strafeAssist);
            mFR.setPower((leftY - leftX) * Math.sin(Math.toRadians(gyroZ) + (Math.PI / 4)) + rightX - strafeAssist);
            mBL.setPower((leftY - leftX) * Math.sin(Math.toRadians(gyroZ) + (Math.PI / 4)) - rightX + strafeAssist);
            mBR.setPower((leftY + leftX) * Math.cos(Math.toRadians(gyroZ) + (Math.PI / 4)) + rightX - strafeAssist);
            */

            /*
            mFL.setPower(leftY - rightX + leftX + strafeAssist);
            mFR.setPower(leftY + rightX - leftX - strafeAssist);
            mBL.setPower(leftY - rightX - leftX + strafeAssist);
            mBR.setPower(leftY + rightX + leftX - strafeAssist);
            */
        }
    }
}