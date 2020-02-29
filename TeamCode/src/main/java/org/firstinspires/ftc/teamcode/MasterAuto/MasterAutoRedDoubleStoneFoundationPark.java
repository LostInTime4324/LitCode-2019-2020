package org.firstinspires.ftc.teamcode.MasterAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.teamcode.BooleanVariable;
import org.firstinspires.ftc.robotcontroller.teamcode.NumberVariable;
import org.firstinspires.ftc.teamcode.OpenCVTest.SkystoneDetector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Park", group="GodBot")


public class MasterAutoRedDoubleStoneFoundationPark extends LinearOpMode {

    //Declares Phone
    private OpenCvInternalCamera phoneCam;
    //Instantiates Skystone Detector using EasyOpenCV
    private SkystoneDetector detector = new SkystoneDetector();
    //Stores the position values
    private String position;

    //
    boolean waffleSide = BooleanVariable.WAFFLE_SIDE.getBoolean();
    boolean skystoneSide = BooleanVariable.SKY_STONE_SIDE.getBoolean();
    boolean doWaffle = BooleanVariable.WAFFLE_OR_NOT.getBoolean();
    boolean parkOrNot = BooleanVariable.PARK_OR_NOT.getBoolean();

    //Servos declared
    public static Servo daHooker;
    public static Servo clawServo;
    public static Servo rightDropper;
    public static Servo leftDropper;
    public static Servo clawServo2;

    //Intakes declared
    public static DcMotor greenWheelLeftIntake;
    public static DcMotor greenWheelRightIntake;

    //The elevator motor declared
    public static DcMotor elevatorTilt;


    @Override
    public void runOpMode() throws InterruptedException {

        if(parkOrNot){

            boolean parkOnTop = BooleanVariable.PARK_AT_TOP.getBoolean();
            boolean parkOnBottom = BooleanVariable.PARK_AT_BOTTOM.getBoolean();

        }
        else{

            boolean parkOnTop = false;
            boolean parkOnBottom = false;

        }

        //Roadrunner REV Optimized Drive Builder
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        Trajectory initialRightAlignment = drive.trajectoryBuilder()
                .lineTo(new Vector2d(0,30), new ConstantInterpolator(0))
                .lineTo(new Vector2d(-6,30), new ConstantInterpolator(0))
                .build();

        Trajectory initialCenterAlignment = drive.trajectoryBuilder()
                .lineTo(new Vector2d(0,30), new ConstantInterpolator(0))
                .lineTo(new Vector2d(-16,30), new ConstantInterpolator(0))
                .build();

        Trajectory initialLeftAlignment = drive.trajectoryBuilder()
                .lineTo(new Vector2d(0,30), new ConstantInterpolator(0))
                .lineTo(new Vector2d(-23,30), new ConstantInterpolator(0))
                .build();

        Trajectory nabStoneForward = drive.trajectoryBuilder()
                .lineTo(new Vector2d(40,0), new ConstantInterpolator(0))
                .build();

        Trajectory nabStoneBack = drive.trajectoryBuilder()
                .lineTo(new Vector2d(-20,0), new ConstantInterpolator(0))
                .build();

        Trajectory nabStoneForwardShort = drive.trajectoryBuilder()
                .lineTo(new Vector2d(30,0), new ConstantInterpolator(0))
                .build();

        Trajectory nabStoneBackShort = drive.trajectoryBuilder()
                .lineTo(new Vector2d(-20,0), new ConstantInterpolator(0))
                .build();

        //All the following are how you name the devices in the configuration
        greenWheelRightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        greenWheelLeftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        daHooker = hardwareMap.get(Servo.class, "daHooker");
        rightDropper = hardwareMap.get(Servo.class, "rightDropper");
        leftDropper = hardwareMap.get(Servo.class, "leftDropper");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo2 = hardwareMap.get(Servo.class,"clawServo2");

//        //Initializes specified phone camera and direction
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//
//        //Starts the OpenCV detector on the phone
//        phoneCam.openCameraDevice();
//        phoneCam.setPipeline(detector);
//
//        //Configures the image on the robot controller
//        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);


        //Pauses the OpMode while detecting the skystone's position
        while (!isStarted()) {
//            position = detector.position;
            telemetry.addData("ready", position);
//            telemetry.update();
        }


        waitForStart();

        drive.followTrajectorySync(

                drive.trajectoryBuilder()

                .back(10)

                .build()
        );



    }

}
