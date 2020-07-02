//TestBed Rover Ruckus
package org.firstinspires.ftc.teamcode.skystone.TB1;

//Imports

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "SensorTest_TB1", group = "TeleOp")
public class SensorTest_TB1 extends BaseClassTB1 {

    int leftEncoder = 0;
    int rightEncoder = 0;
    int centerEncoder = 0;
    double cPos = 0;

    @Override
    public void runOpMode() {

        defineComponents();

        waitForStart();

        while (opModeIsActive()) {

            gyroUpdate();
            updatePoseStrafe();

            //GamePad 1 variables
            double leftX1 = gamepad1.left_stick_x;
            double leftY1 = -gamepad1.left_stick_y;
            double rightX1 = gamepad1.right_stick_x;
            //GamePad 2 variables
            double leftY2 = gamepad2.left_stick_y;
            double rightY2 = gamepad2.right_stick_y;

            leftEncoder = mBL.getCurrentPosition();
            rightEncoder = mFR.getCurrentPosition();
            centerEncoder = mBR.getCurrentPosition();

            telemetry.addData("Left", leftEncoder);
            telemetry.addData("Right", rightEncoder);
            telemetry.addData("Center", centerEncoder);
            telemetry.addData("Gyro", gyroZ);
            telemetry.addData("cPos", cPos);
            telemetry.addData("dI cm", dI.getDistance(DistanceUnit.CM));
            telemetry.addData("X pos", pose.x);
            telemetry.addData("Y pos", pose.y);
            telemetry.addData("", "");
            telemetry.update();

            if(gamepad1.right_bumper) {
                drive(leftY1 / 4, leftX1, rightX1 / 4);
            } else {
                drive(leftY1, leftX1, rightX1);
            }

        }
    }
}