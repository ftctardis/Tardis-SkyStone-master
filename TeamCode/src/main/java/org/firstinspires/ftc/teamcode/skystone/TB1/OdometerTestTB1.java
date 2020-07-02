//TestBed Rover Ruckus
package org.firstinspires.ftc.teamcode.skystone.TB1;

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OdometerTestTB1", group = "TeleOp")
@Disabled

public class OdometerTestTB1 extends BaseClassTB1 {


    @Override
    public void runOpMode() {

        defineComponents();
        sCS.setPosition(0.58);

        double distanceToTargetStart = 0;
        double thetaStart = 0;
        double thetaRobot;

        waitForStart();

        while (opModeIsActive()) {

            gyroUpdate();
            updatePoseStrafe();

            //Parameters
            double targetX = 34.6;
            double targetY = 1.9;
            double targetTheta = 46.3;
            double rotatePercent = 0.4;

            //Distance
            double distanceX = (targetX - pose.x);
            double distanceY = (targetY - pose.y);
            double distanceToTarget = Math.sqrt(Math.pow((distanceX), 2) + Math.pow(distanceY, 2));

            //Powers
            //double strafe = distanceToTarget * Math.cos(-pose.theta - Math.atan(distanceX / distanceY)) / 18;
            double strafe = (2 / (1 + Math.pow(Math.E, -(0.2 * (distanceToTarget * Math.cos(-pose.theta - Math.atan(distanceX / distanceY))))))) - 1;
            //double forward = distanceToTarget * Math.sin(-pose.theta - Math.atan(distanceX / distanceY)) / 18;
            double forward = (2 / (1 + Math.pow(Math.E, -(0.2 * (distanceToTarget * Math.sin(-pose.theta - Math.atan(distanceX / distanceY))))))) - 1;

            //Threshold values for motor power
            if (strafe > 1) {
                strafe = 1;
            } else if (strafe < -1) {
                strafe = -1;
            } else if (strafe < 0.2 && strafe > 0) {
                strafe = 0.2;
            } else if (strafe > -0.2 && strafe < 0) {
                strafe = -0.2;
            }

            if (forward > 1) {
                forward = 1;
            } else if (forward < -1) {
                forward = -1;
            } else if (forward < 0.2 && forward > 0) {
                forward = 0.2;
            } else if (forward > -0.2 && forward < 0) {
                forward = -0.2;
            }

            //Adjust for quadrants
            if (pose.y < targetY) {
                forward = -forward;
                strafe = -strafe;
            }


            //Robot orientation vs distance to target (in degrees)
            if (distanceToTarget > distanceToTargetStart * (1 - rotatePercent)) {
                thetaRobot = (1 - (distanceToTarget - distanceToTargetStart * (1 - rotatePercent)) / (distanceToTargetStart * rotatePercent)) * (targetTheta - thetaStart) + thetaStart;
            } else {
                thetaRobot = targetTheta;
            }

            //Positive rotation = clockwise = decrease in theta
            double rotate = (1 / (1 + Math.pow(Math.E, -(0.13 * (gyroZ - thetaRobot))))) - 0.5;

            //Threshold values for motor power
            if (rotate > 1) {
                rotate = 1;
            } else if (rotate < -1) {
                rotate = -1;
            } else if (rotate < 0.2 && rotate > 0) {
                rotate = 0.2;
            } else if (rotate > -0.2 && rotate < 0) {
                rotate = -0.2;
            }

            telemetry.addLine("Variables");
            telemetry.addData("Theta", gyroZ);
            telemetry.addData("Od X", pose.x);
            telemetry.addData("Od Y", pose.y);
            telemetry.addData("distanceToTarget", distanceToTarget);
            telemetry.addData("drive", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("rotate", rotate);
            telemetry.addData("thetaRobot", thetaRobot);

            telemetry.addLine("\nConstants (press A to reset)");
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("distanceToTargetStart", distanceToTargetStart);
            telemetry.addData("thetaStart",thetaStart);

            telemetry.update();


            //GamePad 1 variables
            double leftX1 = gamepad1.left_stick_x;
            double leftY1 = -gamepad1.left_stick_y;
            double rightX1 = gamepad1.right_stick_x;

            if (gamepad1.a) {
                distanceToTargetStart = distanceToTarget;
                thetaStart = gyroZ;
            }

            if (gamepad1.b && !isInTolerance(targetX, targetY, targetTheta, 1, 2)) {
                drive(forward, strafe, rotate);
            } else {

                //Controls for drive train (Controller 1)
                if (gamepad1.right_bumper) {
                    drive(leftY1 / 4, leftX1 / 2, rightX1 / 4);
                } else {
                    drive(leftY1, leftX1, rightX1);
                }
            }
        }
    }
}