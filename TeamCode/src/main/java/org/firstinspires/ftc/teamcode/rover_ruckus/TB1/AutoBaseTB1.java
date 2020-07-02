package org.firstinspires.ftc.teamcode.rover_ruckus.TB1;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public abstract class AutoBaseTB1 extends BaseClassTB1 {


    int goldPos = 0;
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public static final String VUFORIA_KEY = "Aa9zuMz/////AAAAGTZYlyCroUG9ibBucmCtqD990SI/3cked" +
            "3Q7Nnj4zhBoD72GUWNlfPopl2pvrC3tFmeHYt/I1Rtxubiif5SptIs9SdCheiJZBLEIAOG4nizHWuIB5tU2" +
            "yCjJoO7pNrn/+gn4y2LDgbIQw7AVdp272CbVsp6E/e6zq7fA0Yelv6JN/nLROUo7FWJHz8G98/XL9d1poW9" +
            "D5DlJ++j7ePbgPv+kSPqIhfiQ8uEgtCTLUctdu2/n4M3J/fltvRe4ykRG1UczLleQJ5NMflog4rloogbngG" +
            "NRTVg0DRV0YEQkbnJIcfBz9kDja0Miqvcc6lwPfyBIo591XYi0hU7asnQ2boyWWQGXEdLMnxHQcW5YVmbG";

    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    //Activate Tfod
    public void activateTfod() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    public int getGoldPos() throws NullPointerException {

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() > 3) {
                    /*
                    set updatedRecognitions from the full list but only contains the 2 lowest
                    silver minerals and the lowest gold mineral
                    */
                    int goldMineralY = Integer.MAX_VALUE;
                    int silverMineral1Y = Integer.MAX_VALUE;
                    int silverMineral2Y = Integer.MAX_VALUE;
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        //Find lowest yPos of all gold recognitions
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            if (goldMineralY > recognition.getTop()) {
                                goldMineralY = (int) recognition.getTop();
                                goldMineralX = (int) recognition.getLeft();
                            }
                        }
                        //Find lowest yPos of all silver recognitions
                        if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                            if (silverMineral1Y > recognition.getTop()) {
                                silverMineral1Y = (int) recognition.getTop();
                                silverMineral1X = (int) recognition.getLeft();
                            }
                        }
                        //Find second lowest yPos of all silver recognitions
                        if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                            if (silverMineral2Y > recognition.getTop() && recognition.getTop() > silverMineral1Y) {
                                silverMineral2Y = (int) recognition.getTop();
                                silverMineral2X = (int) recognition.getLeft();
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    goldPos = 1;//Left
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    goldPos = 3;//Right
                                } else {
                                    goldPos = 2;//Center
                                }
                            }
                        }
                    }
                } else {
                    //Not detected
                    //TODO: Update this to work when updatedRecognitions.size() >= 3
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }

                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                goldPos = 1;//Left
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                goldPos = 3;//Right
                            } else {
                                goldPos = 2;//Center
                            }
                        }
                    }
                }
            }
        } else {
            throw new NullPointerException("Tfod not active");
        }
        telemetry.update();
        return goldPos;
    }

    //Shutdown
    public void shutdown() {
        stopDrivetrain();
        stop();
        shutdownTfod();
    }

    public void shutdownTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }


    public enum steps {
        SCAN_MINERALS,
        CLEAR_LANDER,
        FACE_WALL,
        SAMPLE,
        LIFT_ARM,
        KNOCK_SAMPLE,
        ALIGN_RIGHT,
        DRIVE_TO_WALL,
        TURN_TO_DEPOT,
        DRIVE_TO_DEPOT,
        CLAIM,
        DRIVE_TO_CRATER,
        PARK,
        LEAVE_MARKER,
        EXTEND_ARM,
        STOP
    }

    public void gyroAdjust(double degree, double tolerance) {
        if (modernRoboticsI2cGyro.getIntegratedZValue() + tolerance < degree) {
            rotateCounterclockwise(0.2);
        } else if (modernRoboticsI2cGyro.getIntegratedZValue() - tolerance > degree) {
            rotateClockwise(0.2);
        }
    }

    public void exactPosition(double degree, double degTol, double posX, double posY) {
        if (modernRoboticsI2cGyro.getIntegratedZValue() + degTol < degree || modernRoboticsI2cGyro.getIntegratedZValue() - degTol > degree) {
            gyroAdjust(degree, degTol);
        } else {
            if (Math.abs(posX - (int) dR.getDistance(DistanceUnit.CM)) >= Math.abs(posY - (int) dF.getDistance(DistanceUnit.CM))) {
                if (dF.getDistance(DistanceUnit.CM) <= posY) {
                    driveBackward(0.2);
                } else {
                    driveForward(0.2);
                }
            } else {
                if (dR.getDistance(DistanceUnit.CM) <= posX) {
                    driveLeft(0.3);
                } else {
                    driveRight(0.3);
                }
            }
        }
    }

    //Faces the robot to any integrated Z value (degree)
    protected void turnToFace(double direction) {
        while (opModeIsActive() &&
                !isStopRequested() &&
                Math.abs(facingDistanceTo(direction)) > 3.0) {
            if (modernRoboticsI2cGyro.getIntegratedZValue() > direction) {
                rotateClockwise(0.2);
            } else {
                rotateCounterclockwise(0.2);
            }
        }
    }

    /**
     * Given the current facing of the robot and a desired direction, return a number between -180
     * and 180 representing the distance to the direction
     *
     * @param direction
     * @return distance to the direction
     */
    public double facingDistanceTo(double direction) {
        direction = direction % 360;
        double heading = modernRoboticsI2cGyro.getIntegratedZValue() % 360;
        return distanceBetween(heading, direction);
    }

    /**
     * @param a angle a (in degrees)
     * @param b angle b (in degrees)
     * @return the shortest angle ( < 180deg) between angle a and b
     */
    public static double distanceBetween(double a, double b) {
        //a is guaranteed > b
        if (b > a) {
            return -distanceBetween(b, a);
        }

        double diff = a - b;
        if (diff > 180) {
            return -1 * (360 - diff);
        } else {
            return diff;
        }
    }

    //Calibrates gyro with animated telemetry
    public void calibrateGyro() {
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        runtime.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
    }

}