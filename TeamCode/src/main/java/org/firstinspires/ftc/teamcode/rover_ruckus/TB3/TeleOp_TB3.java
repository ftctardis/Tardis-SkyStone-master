//TeleOp Rover Ruckus
package org.firstinspires.ftc.teamcode.rover_ruckus.TB3;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp_TB3", group = "TeleOp")
@Disabled
public class TeleOp_TB3 extends BaseClassTB3 {

    @Override
    public void runOpMode() {

        defineComponents();
        verticalSweeper(0);
        mA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        int atLimit = 0;

        while (opModeIsActive()) {

            //GamePad 1 variables
            double leftX = inverseSigmoid(gamepad1.left_stick_x);
            double leftY = inverseSigmoid(-gamepad1.left_stick_y);
            double rightX = gamepad1.right_stick_x;
            double rightTrigger = gamepad1.right_trigger;
            boolean leftBumper1 = gamepad1.left_bumper;
            double leftTrigger1 = gamepad1.left_trigger;

            //GamePad 2 variables
            float leftY2 = -gamepad2.left_stick_y;
            float rightY2 = -gamepad2.right_stick_y;
            boolean leftStickBtn2 = gamepad2.left_stick_button;
            boolean rightBumper2 = gamepad2.right_bumper;
            boolean leftBumper2 = gamepad1.left_bumper;
            double leftTrigger2 = gamepad2.left_trigger;
            double rightTrigger2 = gamepad2.right_trigger;
            boolean btnX2 = gamepad2.x;
            boolean btnY2 = gamepad2.y;
            boolean btnA2 = gamepad2.a;
            boolean ssLimit = !mlsSS.getState();


            //Telemetry for mA encoder ticks
            telemetry.addData("mA: ", armPos);
            telemetry.addData("mFL: ", mFL.getCurrentPosition());
            telemetry.addData("mBR: ", mBR.getCurrentPosition());
            telemetry.update();

            //Controls for arm raising (Controller 2)
            if (leftY2 > 0) {
                mA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mA.setPower(leftY2 / 1);
                armIsMoving = true;
            } else if (leftY2 < 0) {
                mA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mA.setPower(leftY2 / 3);
                armIsMoving = true;
            } else {
                mAKeepPos();
            }

            if (leftStickBtn2) {
                mA.setPower(0.4);
            }

            //Controls for arm extension (Controller 2)
            mEX.setPower(rightY2);

            //Override
            if (gamepad2.a && !ssLimit) {
                raiseSS();
            } else if (gamepad2.b) {
                lowerSS();
            } else {
                stopSS();
            }

            //Override for Wrist Servo
            /*
            if (leftBumper1) {
                sW.setPosition(0);
            } else if (leftTrigger1 != 0) {
                sW.setPosition(0);
            } else {
                if (mA.getCurrentPosition() < 1200) {
                    sW.setPosition(0);
                } else {
                    sW.setPosition(0);
                }
            }*/

            //Controls for arm deploying (Controller 2)
            if (btnX2) {
                sD.setPosition(0);
            } else if (btnY2) {
                sD.setPosition(0.8);
            }

            //Controls for vertical sweeper (Controller 2)
            if (rightTrigger2 != 0) {
                verticalSweeper(-1);
            } else if (leftTrigger2 != 0) {
                verticalSweeper(1);
            } else {
                verticalSweeper(0);
            }


            //Controls for drive train (Controller 1)
            if (rightTrigger == 0) { //Controls for normal mode (default)
                drive(leftY, leftX, rightX / fastRotate);
            } else { //Controls for slow mode
                drive(leftY / slowDrive, leftX / slowDrive, rightX / slowRotate);
            }
        }
        shutdownTele();
    }
}