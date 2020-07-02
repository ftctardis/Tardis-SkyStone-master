//TeleOp_TB2

//Imports
package org.firstinspires.ftc.teamcode.rover_ruckus.TB2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp_TB2", group = "TeleOp")
@Disabled
public class TeleOp_TB2 extends BaseClassTB2 {

    @Override
    public void runOpMode() {

        //Init functions
        defineComponents();
        verticalSweeper(0);
        mA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive()) {

            //GamePad 1 variables
            double leftX = inverseSigmoid(gamepad1.left_stick_x);
            double leftY = inverseSigmoid(-gamepad1.left_stick_y);
            double rightX = gamepad1.right_stick_x;
            double rightTrigger = gamepad1.right_trigger;

            //GamePad 2 variables
            float leftY2 = -gamepad2.left_stick_y;
            float rightY2 = -gamepad2.right_stick_y;
            boolean leftStickBtn2 = gamepad2.left_stick_button;
            boolean rightBumper2 = gamepad2.right_bumper;
            boolean leftBumper2 = gamepad2.left_bumper;
            double leftTrigger2 = gamepad2.left_trigger;
            double rightTrigger2 = gamepad2.right_trigger;
            boolean btnX2 = gamepad2.x;
            boolean btnY2 = gamepad2.y;

            //Telemetry for mA encoder ticks
            telemetry.addData("mA: ", mA.getCurrentPosition());
            telemetry.update();

            //Controls for arm raising (Controller 2)
            if (leftY2 > 0) {
                mA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mA.setPower(leftY2 / 1.5);
                armIsMoving = true;
            } else if (leftY2 < 0) {
                mA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mA.setPower(leftY2 / 1.75);
                armIsMoving = true;
            } else {
                mAKeepPos();
            }

            //Override for lowering motor
            if (leftStickBtn2) {
                mA.setPower(-0.4);
            }

            //Controls for arm extension (Controller 2)
            mEX.setPower(rightY2);

            //Super structure with limit switch (Controller 2)
            if (!ssIsMoving && leftBumper2){
                //raiseSS
                raiseSS = new Thread(new Runnable(){
                    @Override
                    public void run(){
                        raiseSStoLimit();
                    }
                });
                raiseSS.start();
                ssIsMoving = true;
            }
            if (!ssIsMoving && rightBumper2){
                //lowerSS
                lowerSS = new Thread(new Runnable(){
                    @Override
                    public void run(){
                        lowerSStoLimit();
                    }
                });
                lowerSS.start();
                ssIsMoving = true;
            }
            if (gamepad2.left_bumper) {
                raiseSStoLimit();
            } else if (gamepad2.right_bumper) {
                lowerSStoLimit();
            }

            //Override for super structure (Controller 2)
            if (gamepad2.a) {
                raiseSS();
            } else if (gamepad2.b) {
                lowerSS();
            } else {
                stopSS();
            }

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

            //Controls for rotation of wrist (Controller 2)
            if (mA.getCurrentPosition() > 2000 || gamepad1.left_bumper) {
                sW.setPosition(0.1);
            } else if(mA.getCurrentPosition() < 2000 || gamepad1.left_trigger != 0) {
                sW.setPosition(0.87);
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