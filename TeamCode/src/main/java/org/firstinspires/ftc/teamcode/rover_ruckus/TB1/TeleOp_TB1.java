//TeleOp Rover Ruckus
package org.firstinspires.ftc.teamcode.rover_ruckus.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_TB1", group = "TeleOp")
@Disabled


public class TeleOp_TB1 extends BaseClassTB1 {

    @Override
    public void runOpMode() {

        defineComponents();
        fitInBox();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Magnet: ", mlsSS.getState());
            telemetry.update();

            //GamePad 1 variables
            float leftX = gamepad1.left_stick_x;
            float leftY = -gamepad1.left_stick_y;
            float rightX = -gamepad1.right_stick_x;
            double rightTrigger = gamepad1.right_trigger;

            //GamePad 2 variables
            boolean btnX2 = gamepad2.x;
            boolean btnY2 = gamepad2.y;
            double leftTrigger2 = gamepad2.left_trigger;
            double rightTrigger2 = gamepad2.right_trigger;
            float leftY2 = -gamepad2.left_stick_y;
            float rightY2 = -gamepad2.right_stick_y;


            //Controls for gripper (Controller 2)
            /*if (btnX2) {
                sAngle.setPosition(angleDown);
            } else if (btnY2) {
                sAngle.setPosition(angleUp);
            }*/
            if (leftTrigger2 == 1) {
                sL.setPosition(leftClose);
            } else {
                sL.setPosition(leftOpen);
            }
            if (rightTrigger2 == 1) {
                sR.setPosition(rightClose);
            } else {
                sR.setPosition(rightOpen);
            }


            //Controls for vertical sweeper (Gamepad 2)
            /*if (rightTrigger2 != 0) {
                sR2.setPower(.5);
                sL2.setPower(-.5);
            } else if (leftTrigger2 != 0) {
                sR2.setPower(-.5);
                sL2.setPower(.5);
            } else {
                sR2.setPower(0);
                sL2.setPower(0);
            }*/

            if (leftY2 > 0) {
                mA.setPower((leftY2) / 3 + .15);
            } else {
                mA.setPower((leftY2) / 4 + .1);
            }

            //mEX.setPower(rightY2);

            //Controls for drive train (Controller 1)
            if (rightTrigger == 0) { //Controls for normal mode (default)
                drive(leftY, leftX, rightX / fastRotate);
            } else { //Controls for slow mode
                drive(leftY / slowDrive, leftX / slowDrive, rightX / slowRotate);
            }
        }
    }
}