package org.firstinspires.ftc.teamcode.MasterAuto;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.OpenCVTest.*;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.*;
import org.openftc.easyopencv.*;

//This adds this runnable Autonomous OpMode to the Driver Station
//Add @Disabled under this comment to remove it from the Driver Station
@Autonomous(name = "MasterAutoBlue", group= "GodBot")

//Declares our class that extends LinearOpMode, a built-in class that runs the auto
public class MasterAutoBlue extends LinearOpMode {

    //Declares Phone
    private OpenCvInternalCamera phoneCam;
    //Instantiates Skystone Detector using EasyOpenCV
    private SkystoneDetector detector = new SkystoneDetector();
    //Stores the position values
    private String position;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (!isStarted()) {
            position = detector.position;
            telemetry.addData("position", position);
            telemetry.update();
        }

        waitForStart();

    }
}
