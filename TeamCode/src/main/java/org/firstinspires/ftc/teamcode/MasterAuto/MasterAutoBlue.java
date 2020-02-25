package org.firstinspires.ftc.teamcode.MasterAuto;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcontroller.teamcode.*;
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

        if (parkOrNot) {

            boolean parkOnTop = BooleanVariable.PARK_AT_TOP.getBoolean();
            boolean parkOnBottom = BooleanVariable.PARK_AT_BOTTOM.getBoolean();

        } else {

            boolean parkOnTop = false;
            boolean parkOnBottom = false;

        }

        //Roadrunner REV Optimized Drive Builder
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        //All the following are how you name the devices in the configuration
        greenWheelRightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        greenWheelLeftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        daHooker = hardwareMap.get(Servo.class, "daHooker");
        rightDropper = hardwareMap.get(Servo.class, "rightDropper");
        leftDropper = hardwareMap.get(Servo.class, "leftDropper");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo2 = hardwareMap.get(Servo.class, "clawServo2");

        //Initializes specified phone camera and direction
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Starts the OpenCV detector on the phone
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);

        //Configures the image on the robot controller
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        //Pauses the OpMode while detecting the skystone's position
        while (!isStarted()) {
            position = detector.position;
            telemetry.addData("position", position);
            telemetry.update();
        }

        waitForStart();

        if (waffleSide) {

            if (doWaffle) {


            } else if (parkOrNot) {
            }

        } else if (skystoneSide) {

            if (NumberVariable.NUM_OF_SKY_STONES.getNumber() > 0) {

                if (position == "LEFT") {
                } else if (position == "CENTER") {
                } else if (position == "RIGHT") {
                } else {
                }

                if (NumberVariable.NUM_OF_SKY_STONES_ON_WAFFLE.getNumber() > 0) {


                }

            }

            if (doWaffle) {
            }

            if (NumberVariable.NUM_OF_SKY_STONES.getNumber() > 1) {

                if (position == "LEFT") {
                } else if (position == "CENTER") {
                } else if (position == "RIGHT") {
                } else {
                }


                if (NumberVariable.NUM_OF_SKY_STONES_ON_WAFFLE.getNumber() > 1) {


                }

            }

            if (NumberVariable.NUM_OF_STONES.getNumber() > 0) {

                if (position == "LEFT") {
                } else if (position == "CENTER") {
                } else if (position == "RIGHT") {
                } else {
                }


                if (NumberVariable.NUM_OF_STONES.getNumber() > 0) {


                }

            }

            if (NumberVariable.NUM_OF_STONES.getNumber() > 1) {

                if (position == "LEFT") {
                } else if (position == "CENTER") {
                } else if (position == "RIGHT") {
                } else {
                }


                if (NumberVariable.NUM_OF_STONES.getNumber() > 1) {



                }

            }

            if (parkOrNot) {
            }

        }
    }
}