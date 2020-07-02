//Silver_80_TB2_LSCP
/*
//Imports
package org.firstinspires.ftc.teamcode.skystone.TB0;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
/*
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "RedDepot_7_TB0_DsN", group = "Autonomous")

public class RedDepot_7_TB0_DsN extends AutoBaseTB0 {
/*
    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
        steps CURRENT_STEP = steps.DRIVE_TO_STONES;

        //Init functions
        defineComponents();

        waitForStart();

        //Reset time variables
        runtime.reset();

        //Clear telemetry
        telemetry.clear();

        while (opModeIsActive()) {
///*
            //Update global sensor values
            odometerUpdate();
            gyroUpdate();

            //Telemetry for steps and sensor values
            telemetry.addData("Current Step: ", CURRENT_STEP);
            telemetry.addData("Ticks BR", mBR.getCurrentPosition());
            sensorTelemetry();
            telemetry.update();

            switch (CURRENT_STEP) {

                //Drives into loading zone stone pit
                case DRIVE_TO_STONES:
                    //If driven 8000 ticks forwards
                    if (mBR.getCurrentPosition() > 2500) {
                        resetSteps();
                        CURRENT_STEP = steps.COLLECT_STONE;
                      //Not driven 8000 ticks forward
                    } else {
                        driveForwardGyro(0.3, 0, 2);
                    }
                    break;

                //Rotate towards red alliance bridge, position behind stone
                case COLLECT_STONE:
                    //Rotates clockwise ~-150 degrees
                    if (gyroZ < -120) {
                        resetSteps();
                        CURRENT_STEP = steps.STOP;

                      //Not rotated 140 degrees
                    } else {
                        rotateClockwise(0.25);
                    }
                    break;


                //Parks and collects minerals
                case STOP:
                    stopDriveTrain();
                    break;
            }//
        }
        shutdown();
    }
}
*/