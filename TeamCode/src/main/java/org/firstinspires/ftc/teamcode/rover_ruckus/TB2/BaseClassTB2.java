package org.firstinspires.ftc.teamcode.rover_ruckus.TB2;

//Imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class BaseClassTB2 extends LinearOpMode {

    //Constant variables
    final double slowDrive = 2.5;
    final double slowRotate = 5;
    final double fastRotate = 0.75;

    DcMotor mBL;//Back left
    DcMotor mBR;
    DcMotor mFL;
    DcMotor mFR;//Front right
    DcMotor mA;//Arm
    DcMotor mEX;//Extension
    DcMotor mSS;//Super structure
    Servo sD;//Deploy
    Servo sW; //Servo Wrist
    CRServo sL; //Left
    CRServo sR;
    DistanceSensor dF; //Front
    DistanceSensor dR;
    BNO055IMU imu; //REV gyro
    Orientation angles;
    DigitalChannel mlsSS;//Magnetic limit switch super structure
    ElapsedTime runtime;
    ElapsedTime matchTime;

    Thread raiseSS; //Raising or lowering super structure
    Thread lowerSS;

    float gyroZ;
    boolean ssIsMoving = false;
    boolean armIsMoving = false;
    int armPos = 0;
    Orientation lastAngle = new Orientation();

    public double constantArm() {
        int currPosition = mA.getCurrentPosition();
        double power = -((Math.pow(currPosition - 900, 2)) / (180000000)) + 0.01;
        return power;
    }


    //Defines all components for init()
    public void defineComponents() {

        mBL = hardwareMap.dcMotor.get("mBL");//Back left
        mBR = hardwareMap.dcMotor.get("mBR");
        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");//Front right
        mA = hardwareMap.dcMotor.get("mA");//Arm
        mEX = hardwareMap.dcMotor.get("mEX");//Extension
        mSS = hardwareMap.dcMotor.get("mSS");//Super structure

        mFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sD = hardwareMap.servo.get("sD");//Servo deploy
        sW = hardwareMap.servo.get("sW");//Servo Wrist
        sL = hardwareMap.crservo.get("sL");//Servo left
        sR = hardwareMap.crservo.get("sR");//Servo Right

        dF = hardwareMap.get(DistanceSensor.class, "dF");
        dR = hardwareMap.get(DistanceSensor.class, "dR");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //Calibration file
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        mlsSS = hardwareMap.get(DigitalChannel.class, "mlsSS");

        //Set motors on right side of robot backwards
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mFL.setDirection(DcMotor.Direction.REVERSE);
        mA.setDirection(DcMotor.Direction.REVERSE);
        mSS.setDirection(DcMotor.Direction.REVERSE);

        runtime = new ElapsedTime();
        matchTime = new ElapsedTime();

        verticalSweeper(0);
    }


    //Moves drivetrain in all possible directions
    public void drive(double forward, double strafe, double rotate) {
        mFL.setPower(forward + strafe + rotate);
        mFR.setPower(forward - strafe - rotate);
        mBL.setPower(forward - strafe + rotate);
        mBR.setPower(forward + strafe - rotate);
    }


    public void driveBackward(double power) {
        driveForward(-power);
    }

    public void driveForward(double power) {
        drive(power, 0, 0);
    }

    public void driveLeft(double power) {
        drive(0, -power, 0);
    }

    public void driveRight(double power) {
        driveLeft(-power);
    }


    //Makes sure robot is inside 18" box
    public void fitInBox() {
        //Deploy servo is tucked in
        sD.setPosition(0);
        sW.setPosition(0.89);
        mA.setPower(0);
        //verticalSweeper(-0.1);
    }


    //Updates angles variable
    //public void gyroUpdate() {
    //    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    //}

    public void gyroUpdate() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float originalAngle = angles.firstAngle - lastAngle.firstAngle;

        if (originalAngle < -180) {
            originalAngle += 360;
        } else if (originalAngle > 180) {
            originalAngle -= 360;
        }

        lastAngle = angles;
        gyroZ += originalAngle;

    }


    //Inverse sigmoid function for use in drivetrain directional movement
    public double inverseSigmoid(float input) {
        double value = Math.log((1 - input) / (input + 1)) / -5;
        if (value > 1) {
            value = 1;
        } else if (value < -1) {
            value = -1;
        } else {
            return value;
        }
        return value;
    }


    public void moveSStoOppositeLimit(int raise) {
        ssIsMoving = true;
        ElapsedTime temp = new ElapsedTime();
        boolean isAtLimit;

        do {
            isAtLimit = !mlsSS.getState();
            mSS.setPower(raise);

        } while (opModeIsActive()
                && (gamepad2.left_bumper || gamepad2.right_bumper)
                && (temp.seconds() < 1 || (!isAtLimit)));

        ssIsMoving = false;
    }

    public void lowerSStoLimit() {
        moveSStoOppositeLimit(-1);
    }

    public void raiseSStoLimit() {
        moveSStoOppositeLimit(1);
    }

    public void lowerSS() {
        mSS.setPower(-1);
    }

    public void mAKeepPos() {
        if (armIsMoving) {
            armPos = mA.getCurrentPosition();
        }
        armIsMoving = false;
        mA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mA.setPower(0.5);
        mA.setTargetPosition(armPos);
    }

    public void raiseSS() {
        mSS.setPower(1);
    }


    public void rotateClockwise(double power) {
        drive(0, 0, power);
    }

    public void rotateCounterclockwise(double power) {
        rotateClockwise(-power);
    }

    //Shutdown all processes and stop drivetrain
    public void shutdownTele() {
        stopDriveTrain();
        stop();
        verticalSweeper(0);
    }

    public void stopDriveTrain() {
        drive(0, 0, 0);
    }


    public void stopSS() {
        mSS.setPower(0);
    }


    //Sets both CR servos to run
    public void verticalSweeper(double power) {
        sL.setPower(-power);
        sR.setPower(power);
    }

}