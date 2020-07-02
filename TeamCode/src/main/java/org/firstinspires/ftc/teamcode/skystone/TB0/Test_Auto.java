//Silver_80_TB2_LSCP

//Imports
package org.firstinspires.ftc.teamcode.skystone.TB0;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Test_Auto_SKYSTONE", group = "Autonomous")
@Disabled
public class Test_Auto extends AutoBaseTB0 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.DRIVE_TO_SKYSTONE;
        //Init functions
        defineComponents();
        int targetX = 0;
        int targetY = 0;
        double targetTheta = 0;
        int skystonePosition = 0;

        waitForStart();

        //Reset time variables
        runtime.reset();

        //Clear telemetry
        telemetry.clear();

        while (opModeIsActive()) {

            //Update global sensor values
            odometerUpdate();
            updatePoseStrafe();
            gyroUpdate();

            //Telemetry for steps and sensor values
            telemetry.addData("Current Step: ", CURRENT_STEP);
            telemetry.addData("Pose X: ", pose.x);
            telemetry.addData("Pose Y: ", pose.y);
            telemetry.addData("Pose Theta: ", pose.theta);
            telemetry.addData("Color Sensor Red", c1.red());
            telemetry.addData("Color Sensor Green", c1.green());
            telemetry.addData("Color Sensor Blue", c1.blue());
            telemetry.addData("Skystone Position: ", skystonePosition);
            telemetry.update();

            switch (CURRENT_STEP) {
                case DRIVE_TO_SKYSTONE:
                    targetX = 26;
                    targetY = 0;

                    if (isInTolerance(targetX, targetY, 5)) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_TO_LINE;
                    } else {
                        moveToPose(targetX, targetY, 1, true, 4, true);
                    }
                    break;

                case ROTATE_TO_LINE:
                    if(gyroZ < -75) {
                        changeStep();
                        CURRENT_STEP = steps.DETECT_STONE;
                    } else {
                        rotateSigmoid(-85);
                    }
                    break;

                case DETECT_STONE:
                    if(c1.red() < 25) {
                        if(pose.y > -13) {
                            skystonePosition = 3;
                        } else if(pose.y > -19) {
                            skystonePosition = 2;
                        } else if(pose.y > -26){
                            skystonePosition = 1;
                        }
                        changeStep();
                        CURRENT_STEP = steps.ROTATE_SKYSTONE;
                    } else {
                        driveForwardGyro(0.2, -85, 1);
                    }
                    break;

                case ROTATE_SKYSTONE:
                    if(gyroZ > -70) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    } else {
                        rotateSigmoid(-60);
                    }
                    break;

                case STOP:
                    drive(0, 0, 0);
                    break;
            }
        }
        shutdown();
    }
}