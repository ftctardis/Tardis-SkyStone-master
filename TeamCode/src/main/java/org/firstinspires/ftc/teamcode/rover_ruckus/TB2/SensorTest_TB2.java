//TeleOp Rover Ruckus
package org.firstinspires.ftc.teamcode.rover_ruckus.TB2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "SensorTest_TB2", group = "TeleOp")
@Disabled
public class SensorTest_TB2 extends BaseClassTB2 {

    double encoderAvg = 0;
    double lastEncoder = 0;
    @Override
    public void runOpMode() {

        defineComponents();
        waitForStart();

        int prevDirection = 0;
        boolean prevMagnet;
        double servoValue = 0;

        while (opModeIsActive()) {

            gyroUpdate();
            getEncoderAvg();

            double dFCM = dF.getDistance(DistanceUnit.CM);
            double dRCM = dR.getDistance(DistanceUnit.CM);
            //double gyroZ = angles.firstAngle;
            double mFRTicks = mFR.getCurrentPosition();
            double mBLTicks = mBL.getCurrentPosition();
            double mATicks = mA.getCurrentPosition();
            boolean ssLimit = mlsSS.getState();

            telemetry.addData("dFCM: ", String.format("%.01f cm", dFCM));
            telemetry.addData("dRCM: ", String.format("%.01f cm", dRCM));
            telemetry.addData("Gyro: ", gyroZ);
            telemetry.addData("mFR: ", String.format("%.01f ticks", mFRTicks));
            telemetry.addData("mBL: ", String.format("%.01f ticks", mBLTicks));
            telemetry.addData("mA: ", String.format("%.01f ticks", mATicks));
            telemetry.addData("Encoder avg: ", encoderAvg);
            telemetry.addData("sW Position: ", servoValue);
            telemetry.addData("ssLimit: ", ssLimit);
            telemetry.update();

            /*
            if (gamepad1.x) {
                resetSteps();
            }
            */

            if (gamepad1.a) {
                //servoValue += 0.1;
                sW.setPosition(0.87);
            } else if (gamepad1.b) {
                //servoValue -= 0.1;
                sW.setPosition(0.1);
            }

        }
    }

    public void getEncoderAvg(){
        encoderAvg = (mFR.getCurrentPosition() + mBL.getCurrentPosition()) / 2 - lastEncoder;
    }

    public void resetSteps() {
        lastEncoder = encoderAvg;
        runtime.reset();
    }
}