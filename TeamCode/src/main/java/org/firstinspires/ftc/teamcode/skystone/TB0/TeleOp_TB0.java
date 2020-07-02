//TestBed Rover Ruckus
package org.firstinspires.ftc.teamcode.skystone.TB0;

//Imports

import android.media.MediaActionSound;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleOp_TB0", group = "TeleOp")
@Disabled


public class TeleOp_TB0 extends BaseClassTB0 {

    //int changeEnY;
    int prevEnX = 0;
    int prevEnYLeft = 0;
    int prevEnYRight = 0;
    double prevGyro = 0;
    double enXprediction = 0;

    double odY;
    double odX;

    @Override
    public void runOpMode() {

        defineComponents();

        waitForStart();

        while (opModeIsActive()) {

            gyroUpdate();
            updatePoseStrafe();
            telemetry.addData("mA Encoder Ticks: ", mA.getCurrentPosition());
            telemetry.update();

            //Constants
            //double inchesPerTick = (Math.PI * 4) / 1400;

            //GamePad 1 variables
            double leftX1 = inverseSigmoid(-gamepad1.left_stick_x);
            double leftY1 = inverseSigmoid(gamepad1.left_stick_y);
            double rightX1 = -gamepad1.right_stick_x;
            double rightTrigger1 = gamepad1.right_trigger;
            double leftTrigger1 = gamepad1.left_trigger;
            boolean rightBumper1 = gamepad1.right_bumper;


            //GamePad 2 variables
            float rightTrigger2 = gamepad2.right_trigger;
            float leftTrigger2 = -gamepad2.left_trigger;
            float rightY2 = gamepad2.right_stick_y;
            boolean btnA2 = gamepad2.a;
            boolean btnB2 = gamepad2.b;


            //Controller 1

            //Controls for drivetrain and slow mode
            if(rightBumper1){
                drive(leftY1 / 4, leftX1/ 4, rightX1 / 4);
            } else {
                drive(leftY1, leftX1, rightX1);
            }

            ///////////////////////////////////////////////////////////////////////////////////


            //Controller 2

            //Controls for intake
            if(rightTrigger1 != 0){
                setIntakePower(1);
            } else{
                setIntakePower(0);
            }


            if(leftTrigger1 != 0){
                setIntakePower(-1);
            } else{
                setIntakePower(0);
            }


            //Controls for clamp
            if(leftTrigger2 != 0) {
                sC.setPosition(0.1);
            } else {
                sC.setPosition(1);
            }


            //Controls for arm
            mA.setPower(rightY2);







        }
    }
}