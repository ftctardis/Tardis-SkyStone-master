package org.firstinspires.ftc.teamcode.rover_ruckus.TB2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class ServoTest_TB2 {

    @TeleOp(name = "ServoTest_TB2", group = "TeleOp")
    @Disabled
    public class SensorTest_TB2 extends BaseClassTB2 {

        double servoValue = 0;

        @Override
        public void runOpMode() {

            defineComponents();
            waitForStart();

            while (opModeIsActive()) {


                telemetry.addLine("A = +0.1");
                telemetry.addLine("B = -0.1");
                telemetry.addLine("X = +0.01");
                telemetry.addLine("Y = -0.01");
                telemetry.addData("Servo", servoValue);

                //while (runtime.seconds() > 0.25) {
                    if (gamepad1.a) {
                        servoValue += 0.1;
                    } else if (gamepad1.b) {
                        servoValue -= 0.1;
                    } else if (gamepad1.x) {
                        servoValue += 0.01;
                    } else if (gamepad1.y) {
                        servoValue -= 0.01;
                    }
                //}

                sW.setPosition(servoValue);
            }
        }
    }
}