package org.firstinspires.ftc.teamcode.OpenCVTest;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.openftc.easyopencv.*;


@Autonomous(name = "SkystoneTest", group = "VisionTest")
public class SkystoneTest extends LinearOpMode {
    private OpenCvInternalCamera phoneCam;
    private SkystoneDetector detector = new SkystoneDetector();
    private String position;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

        while (!isStarted()) {
            position = detector.position;
            telemetry.addData("position", position);
            telemetry.update();
        }
    }}