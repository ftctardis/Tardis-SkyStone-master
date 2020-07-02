package org.firstinspires.ftc.teamcode.rover_ruckus.TB2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Servo_15_Sec", group = "Autonomous")
@Disabled

public class Servo_15_Sec extends AutoBaseTB2{


    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        steps CURRENT_STEP = steps.SCAN_MINERALS;

        defineComponents();
        fitInBox();
        initVuforia();
        initTfod();
        activateTfod();

        telemetry.clear();

        //Loop during init to scan for minerals
        while (!opModeIsActive() && !isStopRequested()) {
            getGoldPos();
        }

        waitForStart();

        runtime.reset();
        matchTime.reset();
        telemetry.clear();

        while (opModeIsActive()) {

            gyroUpdate();
            getEncoderAvg();

            double dFCM = dF.getDistance(DistanceUnit.CM);
            double dRCM = dR.getDistance(DistanceUnit.CM);
            double mATicks = mA.getCurrentPosition();
            boolean ssLimit = !mlsSS.getState();

            telemetry.addData("Current Step", CURRENT_STEP);
            telemetry.addData("Matchtime", matchTime.seconds());
            telemetry.addData("Servo Power", sR.getPower());
            sensorTelemetry();
            telemetry.update();

            switch (CURRENT_STEP) {

                case SCAN_MINERALS:
                    if (matchTime.seconds() > 15) {
                        resetSteps();
                        CURRENT_STEP = steps.STOP;
                    } else if (matchTime.seconds() > 10) {
                        verticalSweeper(0);
                    } else {
                        verticalSweeper(0);
                        verticalSweeper(-0.5);
                    }
                    break;


                case STOP:
                    stopDriveTrain();
                    if (matchTime.seconds() > 31) {
                        verticalSweeper(0);
                    } else {
                        verticalSweeper(-0.5);
                    }
                    mA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    mA.setPower(0);
                    shutdownTfod();
                    break;

            }
        }
        shutdown();
    }
}
