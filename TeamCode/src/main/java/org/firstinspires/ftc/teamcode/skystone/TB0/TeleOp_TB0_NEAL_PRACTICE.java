//TestBed Rover Ruckus
package org.firstinspires.ftc.teamcode.skystone.TB0;

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_TB0_Neal", group = "TeleOp")
@Disabled

public class TeleOp_TB0_NEAL_PRACTICE extends BaseClassTB0 {

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

            double leftX = gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y;


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
            telemetry.addData("posX", pose.x);
            telemetry.addData("posY", pose.y);
            telemetry.addData("odY1", -mFL.getCurrentPosition());
            telemetry.addData("odY2", mBL.getCurrentPosition());
            telemetry.addData("pose theta", Math.toDegrees(pose.theta));
            telemetry.update();

            mFL.setPower((leftY + leftX) ); //* Math.cos(Math.toRadians(gyroZ) + (Math.PI / 4)) - rightX + strafeAssist);
            mFR.setPower((leftY - leftX) ); //* Math.sin(Math.toRadians(gyroZ) + (Math.PI / 4)) + rightX - strafeAssist);
            mBL.setPower((leftY - leftX) ); //* Math.sin(Math.toRadians(gyroZ) + (Math.PI / 4)) - rightX + strafeAssist);
            mBR.setPower((leftY + leftX) ); //* Math.cos(Math.toRadians(gyroZ) + (Math.PI / 4)) + rightX - strafeAssist);

            /*
            mFL.setPower(leftY - rightX + leftX + strafeAssist);
            mFR.setPower(leftY + rightX - leftX - strafeAssist);
            mBL.setPower(leftY - rightX - leftX + strafeAssist);
            mBR.setPower(leftY + rightX + leftX - strafeAssist);
            */
        }
    }
}