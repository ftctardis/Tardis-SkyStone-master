//TestBed Rover Ruckus
package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_TB1", group = "TeleOp")
public class TeleOp_TB1 extends BaseClassTB1 {

    //Servo starting positions
    double sWPos = 0.63;
    double sGPos = 0.01; //1
    double sCPos = 0.9;
    double sPPos = 1;
    double sCSPos = 1;

    boolean isStoneTransfer = false;

    @Override
    public void runOpMode() {

        defineComponents();

        waitForStart();

        while (opModeIsActive()) {

            gyroUpdate();

            telemetry.addData("Gantry", sG.getPosition());
            telemetry.addData("Wrist", sW.getPosition());
            telemetry.update();

            //GamePad 1 variables
            double leftX1 = gamepad1.left_stick_x;
            double leftY1 = -gamepad1.left_stick_y;
            double rightX1 = -gamepad1.right_stick_x;
            //GamePad 2 variables
            double leftY2 = gamepad2.left_stick_y;
            double rightY2 = gamepad2.right_stick_y;

            //Mast
            if (leftY2 < 0) {
                mML.setPower(leftY2);
                mMR.setPower(leftY2);
            } else {
                //Slower on way down
                mML.setPower((leftY2 - 0.3) / 10);
                mMR.setPower((leftY2 - 0.3) / 10);
            }


            //Wrist
            if (gamepad2.a) {
                sWPos = 0.28;
            } else if (gamepad2.b && !gamepad2.start) {
                sWPos = 0.63;
            }
            sW.setPosition(sWPos);


            //Gantry
            if (rightY2 < 0) {
                sGPos += 0.06; //-0.2
            } else if (rightY2 > 0) {
                sGPos -= 0.06; //+0.2
            }
            if (sGPos < 0.01) { //0.36
                sGPos = 0.01;
            }
            if (sGPos > 0.99) {
                sGPos = 0.99;
            }
            sG.setPosition(sGPos);


            //Clamp
            if (gamepad2.right_trigger != 0) {
                sCPos = 0.45;
                isStoneTransfer = false;
            } else {
                sCPos = 0.65;
            }
            sC.setPosition(sCPos);


            //Intake
            if (!isStoneTransfer || gamepad1.left_trigger != 0) {
                if (gamepad1.left_trigger != 0) {
                    setIntakePower(1);
                    isStoneTransfer = false;
                } else if (gamepad1.right_trigger != 0) {
                    setIntakePower(-1);
                } else {
                    setIntakePower(0);
                }
            } else {
                setIntakePower(-1);
            }


            //Capstone device
            if (gamepad2.x) {
                sCSPos = 0.2;
                //sGPos = 0.82;
            } else if (gamepad2.y) {
                sCSPos = 1;
            }
            sCS.setPosition(sCSPos);


            //Foundation hooks
            if (gamepad1.a && !gamepad1.start) {
                holdFoundation();
            } else if (gamepad1.b) {
                releaseFoundation();
            }


            //Parking device
            if (gamepad1.x) {
                sPPos = 0.4;
            } else if (gamepad1.y) {
                sPPos = 1;
            }
            sP.setPosition(sPPos);


            //Controls for drive train (Controller 1)
            if (gamepad1.right_bumper) {
                drive(leftY1 / 2, leftX1 / 2, rightX1 / 2);
            } else if (gamepad1.left_bumper) {
                drive(leftY1 / 4, leftX1 / 4, rightX1 / 4);
            } else {
                drive(leftY1, leftX1, rightX1);
            }


            //Automatically powers intake for correct time
            if (isStoneCollected()) {
                isStoneTransfer = true;
            }
            if (!isStoneTransfer) {
                runtime.reset();
            }
            if (runtime.seconds() > 1) {
                sCPos = 0.53;
            }
            if (runtime.seconds() > 1.25) {
                isStoneTransfer = false;
            }
        }
    }
}