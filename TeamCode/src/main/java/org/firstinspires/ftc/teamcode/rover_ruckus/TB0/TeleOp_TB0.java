//TestBed Rover Ruckus
package org.firstinspires.ftc.teamcode.rover_ruckus.TB0;

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp_TB0", group = "TeleOp")
@Disabled
public class TeleOp_TB0 extends OpMode {

    //Defines components
    DcMotor mFL;
    DcMotor mFR;
    DcMotor mBL;
    DcMotor mBR;
    DcMotor mA;
    //DcMotor mC;
    DcMotor mE;
    //Servo sAngle;
    //Servo sUD;
    Servo sL;
    Servo sR;
    //ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    public void init() {

        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");
        mBL = hardwareMap.dcMotor.get("mBL");
        mBR = hardwareMap.dcMotor.get("mBR");
        mA = hardwareMap.dcMotor.get("mA");
        //mC = hardwareMap.dcMotor.get("mC");
        mE = hardwareMap.dcMotor.get("mE");

        sL = hardwareMap.servo.get("sL");
        sR = hardwareMap.servo.get("sR");
        //sAngle = hardwareMap.servo.get("sAngle");
        //sUD = hardwareMap.servo.get("sUD");
        /*
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        modernRoboticsI2cGyro.calibrate(); //Gyro calibration

        while (modernRoboticsI2cGyro.isCalibrating()) { //Adds telemetry for gyro calibration
            telemetry.addData("", "Gyro Calibrating. Please wait..."); //Adds telemetry
            telemetry.update(); //Updates telemetry
        }

        telemetry.addData("", "Gyro Calibrated. Initializing Vuforia..."); //Adds telemetry
        telemetry.update(); //Updates telemetry
        */
        sL.setPosition(1);
        sR.setPosition(0);
        //Reverse motors on right side of robot
        mFR.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.REVERSE);

    }

    //Constant variables
    double slowDrive = 2;
    double slowRotate = 1.5;
    double fastRotate = 0.75;
    double angle = 0;

    @Override
    public void loop() {

        //Fold servo is out
        //sF.setPosition(.7);

        //GamePad 1 variables
        float leftX = -gamepad1.left_stick_x;
        float leftY = -gamepad1.left_stick_y;
        float rightX = gamepad1.right_stick_x;

        //GamePad 2 variables
        float leftY2 = gamepad2.left_stick_y;
        double rightTrigger = gamepad2.right_trigger;
        double leftTrigger = gamepad2.left_trigger;
        boolean buttonA = gamepad2.a;
        boolean buttonB = gamepad2.b;
        boolean rightBumper = gamepad2.right_bumper;
        boolean leftBumper = gamepad2.left_bumper;
        boolean buttonX = gamepad2.x;
        boolean buttonY = gamepad2.y;

        //sAngle.setPosition(angle);

        if(gamepad2.right_trigger != 0) {
            sL.setPosition(0);
        } else {
            sL.setPosition(1);
        }
        if(gamepad2.left_trigger != 0) {
            sR.setPosition(1);
        } else {
            sR.setPosition(0);
        }

        if(gamepad2.left_stick_y > 0) {
            mA.setPower(gamepad2.left_stick_y / 2);
        } else if(gamepad2.left_stick_y < 0){
            mA.setPower(gamepad2.left_stick_y / 2);
        } else {
            mA.setPower(0);
        }


        if(gamepad2.right_stick_y > 0) {
            mE.setPower(gamepad2.right_stick_y / 2);
        } else if(gamepad2.right_stick_y < 0) {
            mE.setPower(gamepad2.right_stick_y / 2);
        } else {
            mE.setPower(0);
        }

        //Controls for drive train (Controller 1)
        if (gamepad1.right_trigger == 0) { //Controls for normal mode (default)
            mFL.setPower((leftX + leftY) + (rightX / fastRotate));
            mFR.setPower((leftY - leftX) - (rightX / fastRotate));
            mBL.setPower((leftY - leftX) + (rightX / fastRotate));
            mBR.setPower((leftX + leftY) - (rightX / fastRotate));
        } else { //Controls for slow mode
            mFL.setPower(((leftX + leftY) / slowDrive) + (rightX / slowRotate));
            mFR.setPower(((leftY - leftX) / slowDrive) - (rightX / slowRotate));
            mBL.setPower(((leftY - leftX) / slowDrive) + (rightX / slowRotate));
            mBR.setPower(((leftX + leftY) / slowDrive) - (rightX / slowRotate));
        }
    }
}