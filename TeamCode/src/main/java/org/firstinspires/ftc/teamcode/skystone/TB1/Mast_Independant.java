//TestBed Rover Ruckus
package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mast_Independant", group = "TeleOp")
@Disabled

public class Mast_Independant extends BaseClassTB1 {

    //Servo starting positions 
    double sWPos = 0.41;
    double sGPos = 1;
    double sCPos = 0;
    double sPPos = 1;
    double sCSPos = 0.05;

    @Override
    public void runOpMode() {

        defineComponents();
        sCS.setPosition(0.05);

        waitForStart();

        while (opModeIsActive()) {

            gyroUpdate();

            telemetry.addLine("Gamepad 1 joysticks control left and right mast motors");
            telemetry.update();

            //GamePad 1 variables
            double leftY1 = gamepad1.left_stick_y;
            double rightY1 = gamepad1.right_stick_y;
            //GamePad 2 variables
            double leftY2 = gamepad2.left_stick_y;
            double rightY2 = gamepad2.right_stick_y;

            //Mast
            if (leftY2 < 0) {
                mML.setPower(leftY2 + leftY1);
                mMR.setPower(leftY2 + rightY1); //Decrease power by 15%
            } else {
                mML.setPower((leftY2 + leftY1 - 0.2) / 4);
                mMR.setPower((leftY2 + rightY1 - 0.2) / 4);
            }

            //Wrist
            if (gamepad2.a) {
                sWPos = 0.41;
            } else if (gamepad2.b && !gamepad2.start) {
                sWPos = 0.28;
            }
            sW.setPosition(sWPos);

            //Gantry
            if (gamepad2.right_stick_y < 0) {
                sGPos -= 0.02;
            } else if (gamepad2.right_stick_y > 0) {
                sGPos += 0.02;
            }

            if (sGPos < 0.36) {
                sGPos = 0.36;
            }

            if (sGPos > 1) {
                sGPos = 1;
            }
            sG.setPosition(sGPos);

            //Clamp
            if (gamepad2.right_trigger != 0) {
                sCPos = 0.45;
            } else {
                sCPos = 0.7;
            }
            sC.setPosition(sCPos);

            //Intake
            if (gamepad1.left_trigger != 0) {
                setIntakePower(1);
            } else if (gamepad1.right_trigger != 0) {
                setIntakePower(-1);
            } else {
                setIntakePower(0);
            }

            //Capstone device
            if (gamepad2.x) {
                sCSPos = 0.58;
                sGPos = 0.82;
            } else if (gamepad2.y) {
                sCSPos = 0.05;
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
        }
    }
}