package org.firstinspires.ftc.teamcode.skystone.TB0;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "DetectSkystones", group = "Autonomous")
@Disabled
public class DetectSkystones extends LinearOpMode {
    //Variables for tuning position
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;
    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;
    private static float offsetX = 0f / 8f;
    private static float offsetY = 0f / 8f;
    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
    private final int rows = 640;
    private final int cols = 480;
    OpenCvCamera phoneCam;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //Setup camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//Open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//Display image on robot controller

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Skystone position", skystonePosition());
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            telemetry.update();
        }
    }

    //Return string describing position of skystones
    public String skystonePosition() {
        /**
         * returns 0 if left, 1 if center, 2 if right, 3 if none
         */
        if (valLeft < valRight && valLeft < valMid) {
            return "left";
            //Return left
        } else if (valMid < valRight && valMid < valLeft) {
            return "middle";
            //Return middle
        } else if (valRight < valLeft && valRight < valMid) {
            return "right";
            //Return right
        } else {
            return "unknown";
            //Return none
        }
    }

    //Detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline {
        //Defines matrices
        Mat yCrCbChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        private Stage stageToRenderToViewport = Stage.RAW_IMAGE;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {

            //Cycle through stages
            int currentStage = stageToRenderToViewport.ordinal();
            int nextStage = currentStage + 1;

            if (nextStage >= stages.length) {
                nextStage = 0;
            }
            stageToRenderToViewport = stages[nextStage];
        }

        @Override
        public Mat processFrame(Mat input) {

            //Default all white
            int leftColor = 255;
            int midColor = 255;
            int rightColor = 255;

            //Make the skystone box purple
            if (valLeft < valRight && valLeft < valMid) {
                leftColor = 0;
                midColor = 255;
                rightColor = 255;
                //Return left
            } else if (valMid < valRight && valMid < valLeft) {
                leftColor = 255;
                midColor = 0;
                rightColor = 255;
                //Return middle
            } else if (valRight < valLeft && valRight < valMid) {
                leftColor = 255;
                midColor = 255;
                rightColor = 0;
                //Return right
            }

            //Find contours of yellow objects
            contoursList.clear();
            Mat shapesMat = input;

            Imgproc.cvtColor(input, yCrCbChan2Mat, Imgproc.COLOR_RGB2YCrCb);//Converts rgb input to yCrCb
            Core.extractChannel(yCrCbChan2Mat, yCrCbChan2Mat, 2);//Extracts Cb (blue yellow) channel to threshold
            Imgproc.threshold(yCrCbChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);//Threshold at a Cb value of 102

            //Resets sum values to 0 before each iteration
            valMid = 0;
            valLeft = 0;
            valRight = 0;

            //Sums values of an 11 pixel square from midpoint
            for (int pixoffsetX = -5; pixoffsetX <= 5; pixoffsetX++) {
                for (int pixoffsetY = -5; pixoffsetY <= 5; pixoffsetY++) {
                    double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]) + pixoffsetY, (int) (input.cols() * midPos[0]) + pixoffsetX);
                    valMid += (int) pixMid[0];
                    double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]) + pixoffsetY, (int) (input.cols() * leftPos[0]) + pixoffsetX);
                    valLeft += (int) pixLeft[0];
                    double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]) + pixoffsetY, (int) (input.cols() * rightPos[0]) + pixoffsetX);
                    valRight += (int) pixRight[0];
                }
            }

            //Draw squares to represent pixel sums
            Imgproc.rectangle(
                    shapesMat,
                    new Point((int) (input.cols() * leftPos[0]) - 5, (int) (input.rows() * leftPos[1]) - 5),
                    new Point((int) (input.cols() * leftPos[0]) + 5, (int) (input.rows() * leftPos[1]) + 5),
                    new Scalar(255, leftColor, 255), 1);
            Imgproc.rectangle(
                    shapesMat,
                    new Point((int) (input.cols() * midPos[0]) - 5, (int) (input.rows() * midPos[1]) - 5),
                    new Point((int) (input.cols() * midPos[0]) + 5, (int) (input.rows() * midPos[1]) + 5),
                    new Scalar(255, midColor, 255), 1);
            Imgproc.rectangle(
                    shapesMat,
                    new Point((int) (input.cols() * rightPos[0]) - 5, (int) (input.rows() * rightPos[1]) - 5),
                    new Point((int) (input.cols() * rightPos[0]) + 5, (int) (input.rows() * rightPos[1]) + 5),
                    new Scalar(255, rightColor, 255), 1);

            //Draw rectangles for boundary of stones
            Imgproc.rectangle(
                    shapesMat,
                    new Point(input.cols() * (leftPos[0] - rectWidth / 2), input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(input.cols() * (leftPos[0] + rectWidth / 2), input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(255, leftColor, 255), 2);
            Imgproc.rectangle(
                    shapesMat,
                    new Point(input.cols() * (midPos[0] - rectWidth / 2), input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(input.cols() * (midPos[0] + rectWidth / 2), input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(255, midColor, 255), 2);
            Imgproc.rectangle(
                    shapesMat,
                    new Point(input.cols() * (rightPos[0] - rectWidth / 2), input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(input.cols() * (rightPos[0] + rectWidth / 2), input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(255, rightColor, 255), 2);

            //Cases for tap pipeline
            switch (stageToRenderToViewport) {

                case THRESHOLD: {
                    return thresholdMat;
                }
                default: {
                    return input;
                }
            }
        }

        //Stages for tap pipeline
        enum Stage {
            THRESHOLD,
            RAW_IMAGE,
        }
    }
}
