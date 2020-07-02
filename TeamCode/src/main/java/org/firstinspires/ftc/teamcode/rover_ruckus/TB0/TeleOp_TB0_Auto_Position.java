//TestBed Rover Ruckus
package org.firstinspires.ftc.teamcode.rover_ruckus.TB0;

//Imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleOp_TB0_Auto_Pos", group = "TeleOp")
@Disabled
public class TeleOp_TB0_Auto_Position extends OpMode {

    //Defines components
    DcMotor mFL;
    DcMotor mFR;
    DcMotor mBL;
    DcMotor mBR;
    BNO055IMU imu; //REV gyro

    Orientation angles;
    float gyroZ;
    Orientation lastAngle = new Orientation();

    int mFRCurrPos;
    int mFLCurrPos;
    int mBRCurrPos;
    int mBLCurrPos;

    boolean xPressed = false;
    boolean xPressedPrev = false;
    boolean yPressed = false;
    boolean yPressedPrev = false;
    boolean aPressed = false;
    boolean aPressedPrev = false;
    boolean bPressed = false;
    boolean bPressedPrev = false;


    String field[][] = new String[12][12];

    public void init() {

        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");
        mBL = hardwareMap.dcMotor.get("mBL");
        mBR = hardwareMap.dcMotor.get("mBR");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //Calibration file
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Reverse motors on right side of robot
        mFL.setDirection(DcMotor.Direction.REVERSE);
        mBL.setDirection(DcMotor.Direction.REVERSE);

        //Init Field
        for(int x = 0; x < 12; x++){
            for(int y = 0; y < 12; y++) {
                field[y][x] = "0 ";
            }
        }
    }

    AutoStep StepOne = new AutoStep();

    @Override
    public void loop() {
        gyroUpdate();
        motorEncoderUpdate();
        //telemtryUpdate();
        autoUpdate(StepOne);

        xPressed = gamepad1.x; //Rename
        yPressed = gamepad1.y;

        if(xPressed && !xPressedPrev) {
            StepOne.initialize(mFLCurrPos, mFRCurrPos, mBLCurrPos, mBRCurrPos, gyroZ);
        }

        if(yPressed && !yPressedPrev) {
            StepOne.finalize(mFLCurrPos, mFRCurrPos, mBLCurrPos, mBRCurrPos, gyroZ);
        }

        if(aPressed && !aPressedPrev) {
            mFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mFL.setTargetPosition(StepOne.startingFL);
            mFR.setTargetPosition(StepOne.startingFR);
            mBL.setTargetPosition(StepOne.startingBL);
            mBR.setTargetPosition(StepOne.startingBR);
            mFL.setPower(0.5);
            mFR.setPower(0.5);
            mBL.setPower(0.5);
            mBR.setPower(0.5);
        }

        if(bPressed && !bPressedPrev) {
            mFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mFL.setTargetPosition(StepOne.endingFL);
            mFR.setTargetPosition(StepOne.endingFR);
            mBL.setTargetPosition(StepOne.endingBL);
            mBR.setTargetPosition(StepOne.endingBR);
            mFL.setPower(0.5);
            mFR.setPower(0.5);
            mBL.setPower(0.5);
            mBR.setPower(0.5);
        }

        double speed_x = -gamepad1.left_stick_x;
        double speed_y = gamepad1.left_stick_y;
        double speed_r = -gamepad1.right_stick_x;

        mFL.setPower(speed_y + speed_x + speed_r);
        mFR.setPower(speed_y - speed_x - speed_r);
        mBL.setPower(speed_y - speed_x + speed_r);
        mBR.setPower(speed_y + speed_x - speed_r);

        //Driving while rotating
        //mFL.setPower((speed_y + speed_x) * Math.sin(degToRad(gyroZ) + (Math.PI / 4)) + speed_r);
        //mFR.setPower((speed_y - speed_x) * Math.cos(degToRad(gyroZ) + (Math.PI / 4)) - speed_r);
        //mBL.setPower((speed_y - speed_x) * Math.cos(degToRad(gyroZ) + (Math.PI / 4)) + speed_r);
        //mBR.setPower((speed_y + speed_x) * Math.sin(degToRad(gyroZ) + (Math.PI / 4)) - speed_r);

        xPressedPrev = xPressed;
        yPressedPrev = yPressed;

    }

    public void autoUpdate(AutoStep step) {
        telemetry.addData("Starting Orientation ", step.startingOrientation);
        telemetry.addData("mFL Start ", step.startingFL);
        telemetry.addData("mFR Start ", step.startingFR);
        telemetry.addData("mBL Start ", step.startingBL);
        telemetry.addData("mBR Start ", step.startingBR);
        telemetry.addData("--------", "--------");
        telemetry.addData("Ending Orientation ", step.endingOrientation);
        telemetry.addData("mFL End ", step.endingFL);
        telemetry.addData("mFR End ", step.endingFR);
        telemetry.addData("mBL End ", step.endingBL);
        telemetry.addData("mBR End ", step.endingBR);
        telemetry.update();
    }

    public void motorEncoderUpdate() {
        mFRCurrPos = mFR.getCurrentPosition() * -1;
        mBRCurrPos = mBR.getCurrentPosition() * -1;
        mFLCurrPos = mFL.getCurrentPosition() * -1;
        mBLCurrPos = mBL.getCurrentPosition() * -1;
    }

    public void displayField() {
        for(int y = 0; y < 12; y++) {
            String row = "";
            String rowNum = String.valueOf(y);
            for(int x = 0; x < 12; x++) {
                row += field[y][x];
            }
            telemetry.addData(rowNum, row);
        }
    }

    public void telemtryUpdate() {
        displayField();
        telemetry.addData("Gyro", gyroZ);
        telemetry.addData("mFL encoder ticks ", mFLCurrPos);
        telemetry.addData("mFR encoder ticks ", mFRCurrPos);
        telemetry.addData("mBL encoder ticks ", mBLCurrPos);
        telemetry.addData("mBR encoder ticks ", mBRCurrPos);
        telemetry.update();
    }

    public double degToRad(float x) {
        return x * (Math.PI / 180);
    }

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

    public class AutoStep {
        int startingFL;
        int startingFR;
        int startingBL;
        int startingBR;
        float startingOrientation;

        int endingFL;
        int endingFR;
        int endingBL;
        int endingBR;
        float endingOrientation;

        public AutoStep() {
            this.startingFL = -1;
            this.startingFR = -1;
            this.startingBL = -1;
            this.startingBR = -1;
            this.startingOrientation = -1;

            this.endingFL = -1;
            this.endingFR = -1;
            this.endingBL = -1;
            this.endingBR = -1;
            this.endingOrientation = -1;
        }
        public void initialize(int startingFL, int startingFR, int startingBL, int startingBR, float startingOrientation) {
            this.startingFL = startingFL;
            this.startingFR = startingFR;
            this.startingBL = startingBL;
            this.startingBR = startingBR;
            this.startingOrientation = startingOrientation;
        }
        public void finalize(int endingFL, int endingFR, int endingBL, int endingBR, float endingOrientation) {
            this.endingFL = endingFL;
            this.endingFR = endingFR;
            this.endingBL = endingBL;
            this.endingBR = endingBR;
            this.endingOrientation = endingOrientation;
        }
    }
}