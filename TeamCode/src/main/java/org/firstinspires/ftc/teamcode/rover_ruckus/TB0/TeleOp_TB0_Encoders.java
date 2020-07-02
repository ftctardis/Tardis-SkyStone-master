//TestBed Rover Ruckus
package org.firstinspires.ftc.teamcode.rover_ruckus.TB0;

//Imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleOp_TB0_Encoders", group = "TeleOp")
@Disabled

public class TeleOp_TB0_Encoders extends OpMode {

    //Defines components
    DcMotor mFL;
    DcMotor mFR;
    DcMotor mBL;
    DcMotor mBR;

    DcMotor mEnX;
    DcMotor mEnY;

    BNO055IMU imu; //REV gyro
    Orientation angles;
    float gyroZ;
    Orientation lastAngle = new Orientation();
    int enXPrev = 0;
    int enYPrev = 0;
    float changeInPosX;
    float changeInPosY;
    float posX;
    float posY;
    int mFRCurrPos;
    int mFLCurrPos;
    int mBRCurrPos;
    int mBLCurrPos;

    public void init() {

        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");
        mBL = hardwareMap.dcMotor.get("mBL");
        mBR = hardwareMap.dcMotor.get("mBR");

        //mEnX = hardwareMap.dcMotor.get("mEnX");
        //mEnY = hardwareMap.dcMotor.get("mEnY");

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

        //mEnX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mEnX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        gyroUpdate();
        odometerUpdate();
        motorEncoderUpdate();
        telemtryUpdate();

        //telemetry.addData("Encoder X", mEnX.getCurrentPosition());
        //telemetry.addData("Encoder Y", mEnY.getCurrentPosition());

        double speed_x = gamepad1.left_stick_y;
        double speed_r = gamepad1.right_stick_x;

        mFL.setPower(speed_x * Math.sin(degToRad(gyroZ) + (Math.PI / 4)) + speed_r);
        mFR.setPower(speed_x * Math.cos(degToRad(gyroZ) + (Math.PI / 4)) - speed_r);
        mBL.setPower(speed_x * Math.cos(degToRad(gyroZ) + (Math.PI / 4)) + speed_r);
        mBR.setPower(speed_x * Math.sin(degToRad(gyroZ) + (Math.PI / 4)) - speed_r);
    }

    public void motorEncoderUpdate() {
        mFRCurrPos = mFR.getCurrentPosition() * -1;
        mBRCurrPos = mBR.getCurrentPosition() * -1;
        mFLCurrPos = mFL.getCurrentPosition() * -1;
        mBLCurrPos = mBL.getCurrentPosition() * -1;
    }

    public void telemtryUpdate() {
        telemetry.addData("Gyro", gyroZ);
        telemetry.addData("Pos X", posX);
        telemetry.addData("Pos Y", posY);
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

    public void odometerUpdate() {

        //int enX = mEnX.getCurrentPosition();
        //int enY = mEnY.getCurrentPosition();
        /*
        if (Math.abs(Math.acos(gyroZ)) > Math.abs(Math.asin(gyroZ))) {
            changeInPosX = ((float)enX - (float)enXPrev) * (float)Math.acos(gyroZ);
            changeInPosY = ((float)enY - (float)enYPrev) * (float)Math.acos(gyroZ);
        } else {
            changeInPosX = ((float)enX - (float)enXPrev) * (float)Math.asin(gyroZ);
            changeInPosY = ((float)enY - (float)enYPrev) * (float)Math.asin(gyroZ);
        }
        posX += changeInPosX;
        posY += changeInPosY;

        enXPrev = enX;
        enYPrev = enY;
        */

        //posX = enX;
        //posY = enY;
    }

    /*
    public void drive(double forward, double strafe, double rotate) {
        mFL.setPower(forward + strafe + rotate);
        mFR.setPower(forward - strafe - rotate);
        mBL.setPower(forward - strafe + rotate);
        mBR.setPower(forward + strafe - rotate);
    }
    */
}