package org.firstinspires.ftc.teamcode.OpenCVTest;

import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.openftc.easyopencv.*;

public class SkystoneDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public String position = "LEFT";
    public SkystoneDetector() {

    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(120, 150, 10, 50);
        Mat matCenter = workingMatrix.submat(120, 150, 80, 120);
        Mat matRight = workingMatrix.submat(120, 150, 150, 190);

        Imgproc.rectangle(workingMatrix, new Rect(10, 120, 40, 30), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(80, 120, 40, 30), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(150, 120, 40, 30), new Scalar(0,255,0));
//
//        Mat matLeft = workingMatrix.submat(120, 150, 10, 50);
//        Mat matCenter = workingMatrix.submat(120, 150, 80, 120);
//        Mat matRight = workingMatrix.submat(120, 150, 150, 190);
//
//        Imgproc.rectangle(workingMatrix, new Rect(120, 10, 40, 30), new Scalar(0,255,0));
//        Imgproc.rectangle(workingMatrix, new Rect(120, 80, 40, 30), new Scalar(0,255,0));
//        Imgproc.rectangle(workingMatrix, new Rect(120, 150, 40, 30), new Scalar(0,255,0));

        double leftTotal = Core.sumElems(matLeft).val[2];
        double centerTotal = Core.sumElems(matCenter).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];

        if (leftTotal > centerTotal) {
            if (leftTotal > rightTotal) {
                // left is skystone
                position = "LEFT";
            } else {
                // right is skystone
                position = "RIGHT";
            }
        } else {
            if (centerTotal > rightTotal) {
                // center is skystone
                position = "CENTER";
            } else {
                // right is skystone
                position = "RIGHT";
            }
        }

        return workingMatrix;
    }
}
