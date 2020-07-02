package org.firstinspires.ftc.teamcode.rover_ruckus.TB1;

//Imports

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public abstract class BaseClassTB1 extends LinearOpMode {

    //Constant variables
    final double slowDrive = 2;
    final double slowRotate = 1.5;
    final double fastRotate = 0.9;

    final double leftOpen = 0;
    final double leftClose = 1;//0.3
    final double rightOpen = 1;//0.7
    final double rightClose = 0;//0.3
    final double angleUp = 1;
    final double angleDown = 0;

    DcMotor mBL; //Back left
    DcMotor mBR;
    DcMotor mFL;
    DcMotor mFR; //Front right
    DcMotor mA; //Arm
    DcMotor mEX; //Extension

    Servo sL;
    Servo sR;
    Servo sAngle;

    CRServo sL2;
    CRServo sR2;

    DistanceSensor dF;
    DistanceSensor dR;

    DigitalChannel mlsSS;//Magnetic limit switch super structure

    IntegratingGyroscope gyro; //Gyro
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    ElapsedTime runtime;

    public void drive(double forward, double strafe, double rotate) {
        mFL.setPower(forward + strafe + rotate);
        mFR.setPower(forward - strafe - rotate);
        mBL.setPower(forward - strafe + rotate);
        mBR.setPower(forward + strafe - rotate);
    }

    public void rotateClockwise(double power) {
        drive(0, 0, power);
    }

    public void rotateCounterclockwise(double power) {
        rotateClockwise(-power);
    }

    public void driveForward(double power) {
        drive(power, 0, 0);
    }

    public void driveBackward(double power) {
        driveForward(-power);
    }

    public void driveLeft(double power) {
        drive(0, -power, 0);
    }

    public void driveRight(double power) {
        driveLeft(-power);
    }

    public void openGripper() {
        sL.setPosition(leftOpen);
        sR.setPosition(rightOpen);
    }

    public void closeGripper() {
        sL.setPosition(leftClose);
        sR.setPosition(rightClose);
    }

    public void stopDrivetrain() {
        drive(0, 0, 0);
    }

    //Defines all components for init()
    public void defineComponents() {

        mBL = hardwareMap.dcMotor.get("mBL");//Back left
        mBR = hardwareMap.dcMotor.get("mBR");
        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");//Front right
        mA = hardwareMap.dcMotor.get("mA");//Arm
        mEX = hardwareMap.dcMotor.get("mEX");//Extension

        sL = hardwareMap.servo.get("sL");//Servo left
        sR = hardwareMap.servo.get("sR");

        sL2 = hardwareMap.crservo.get("sL2");
        sL2.setDirection(DcMotorSimple.Direction.FORWARD);
        sR2 = hardwareMap.crservo.get("sR2");
        sR2.setDirection(DcMotorSimple.Direction.FORWARD);

        sAngle = hardwareMap.servo.get("sAngle");

        dF = hardwareMap.get(DistanceSensor.class, "dF");
        dR = hardwareMap.get(DistanceSensor.class, "dR");

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        mlsSS = hardwareMap.get(DigitalChannel.class, "mlsSS");

        //Set motors on right side of robot backwards
        mBR.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.REVERSE);

        runtime = new ElapsedTime();
    }

    public void fitInBox() {
        //Fold servo is tucked in
        sL.setPosition(leftClose);
        sR.setPosition(rightClose);

        sL2.setPower(0);
        sR2.setPower(0);
    }
}