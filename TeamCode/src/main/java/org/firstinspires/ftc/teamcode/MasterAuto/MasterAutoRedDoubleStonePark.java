package org.firstinspires.ftc.teamcode.MasterAuto;

import com.acmerobotics.roadrunner.drive.*;
import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.path.heading.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcontroller.teamcode.*;
import org.firstinspires.ftc.teamcode.OpenCVTest.*;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.*;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.*;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode.*;
import org.openftc.easyopencv.*;

//This adds this runnable Autonomous OpMode to the Driver Station
//Add @Disabled under this comment to remove it from the Driver Station
@Autonomous(name="MasterAutoRed", group="GodBot")

//Declares our class that extends LinearOpMode, a built-in class that runs the auto
public class MasterAutoRedDoubleStonePark extends LinearOpMode {

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

        //Initializes specified phone camera and direction
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Starts the OpenCV detector on the phone
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);

        //Configures the image on the robot controller
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);


        //Pauses the OpMode while detecting the skystone's position
        while (!isStarted()) {
            position = detector.position;
            telemetry.addData("position", position);
            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;



            if(NumberVariable.NUM_OF_SKY_STONES.getNumber() > 0){



                if(position == "LEFT"){

                    drive.followTrajectorySync(initialLeftAlignment);

                    greenWheelLeftIntake.setPower(-1);

                    greenWheelRightIntake.setPower(0.5);

                    drive.turnSync(Math.toRadians(90));
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(nabStoneForwardShort);
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(nabStoneBackShort);
                    drive.turnSync(Math.toRadians(-90));


                    drive.setPoseEstimate(new Pose2d(0,0,0));
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(65,0), new ConstantInterpolator(0))
                                    .build()

                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                }

                else if(position == "CENTER"){

                    drive.followTrajectorySync(initialCenterAlignment);


                    greenWheelLeftIntake.setPower(-1.0);


                    greenWheelRightIntake.setPower(0.5);
                    drive.turnSync(Math.toRadians(90));

                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(nabStoneForwardShort);
                    drive.setPoseEstimate(new Pose2d(0,0,0));


                    drive.followTrajectorySync(nabStoneBackShort);
                    drive.turnSync(Math.toRadians(-90));


                    drive.setPoseEstimate(new Pose2d(0,0,0));
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(55,0), new ConstantInterpolator(0))
                                    .build()

                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));



                }

                else if(position == "RIGHT"){


                    drive.followTrajectorySync(initialRightAlignment);

                    greenWheelLeftIntake.setPower(-1);

                    greenWheelRightIntake.setPower(1);
                    drive.turnSync(Math.toRadians(90));

                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(nabStoneForward);

                    drive.setPoseEstimate(new Pose2d(0,0,0));


                    drive.followTrajectorySync(nabStoneBack);
                    drive.turnSync(Math.toRadians(-90));

                    drive.setPoseEstimate(new Pose2d(0,0,0));
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(50,0), new ConstantInterpolator(0))
                                    .build()

                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));






                }

                else {
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(0,30), new ConstantInterpolator(0))
                                    .build()
                    );
                    greenWheelLeftIntake.setPower(-1.0);

                    greenWheelRightIntake.setPower(1.0);
                    drive.turnSync(Math.toRadians(120));
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(50,0), new ConstantInterpolator(0))
                                    .build()

                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(-10,0), new ConstantInterpolator(0))
                                    .build()

                    );
                    drive.turnSync(Math.toRadians(-120));
                    drive.setPoseEstimate(new Pose2d(0,0,0));
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(30,0), new ConstantInterpolator(0))
                                    .build()

                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                }




                drive.setPoseEstimate(new Pose2d(0,0,0));

                greenWheelLeftIntake.setPower(1.0);

                greenWheelRightIntake.setPower(-1.0);

                drive.turnSync(Math.toRadians(180));

                drive.setPoseEstimate(new Pose2d(0,0,0));




            }


            if(NumberVariable.NUM_OF_SKY_STONES.getNumber() >= 1){

                if(position == "LEFT"){

                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(80,0), new ConstantInterpolator(0))
                                    .build()
                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(0,-40), new ConstantInterpolator(0))
                                    .build()
                    );

                    greenWheelLeftIntake.setPower(-1);

                    greenWheelRightIntake.setPower(1);

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(10,0), new ConstantInterpolator(0))
                                    .build()
                    );

                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(0,25), new ConstantInterpolator(0))
                                    .build()
                    );


                    drive.turnSync(Math.toRadians(-180));
                    drive.setPoseEstimate(new Pose2d(0,0,0));
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(80,0), new ConstantInterpolator(0))
                                    .build()

                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));


                }

                else if(position == "CENTER"){
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(65,0), new ConstantInterpolator(0))
                                    .build()
                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(0,-40), new ConstantInterpolator(0))
                                    .build()
                    );

                    greenWheelLeftIntake.setPower(-1);

                    greenWheelRightIntake.setPower(1);

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(10,0), new ConstantInterpolator(0))
                                    .build()
                    );

                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(0,25), new ConstantInterpolator(0))
                                    .build()
                    );


                    drive.turnSync(Math.toRadians(-180));
                    drive.setPoseEstimate(new Pose2d(0,0,0));
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(65,0), new ConstantInterpolator(0))
                                    .build()

                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                }

                else if(position == "RIGHT"){
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(60,0), new ConstantInterpolator(0))
                                    .build()
                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(0,-40), new ConstantInterpolator(0))
                                    .build()
                    );

                    greenWheelLeftIntake.setPower(-1);

                    greenWheelRightIntake.setPower(1);

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(5,0), new ConstantInterpolator(0))
                                    .build()
                    );

                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(0,25), new ConstantInterpolator(0))
                                    .build()
                    );


                    drive.turnSync(Math.toRadians(-180));
                    drive.setPoseEstimate(new Pose2d(0,0,0));
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(60,0), new ConstantInterpolator(0))
                                    .build()

                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                }

                else {
                    drive.setPoseEstimate(new Pose2d(0,0,0));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(60,0), new ConstantInterpolator(0))
                                    .build()
                    );
                    drive.turnSync(Math.toRadians(-90));
                    drive.setPoseEstimate(new Pose2d(0,0,0));

//                    drive.followTrajectorySync(nabStone);

                    drive.turnSync(Math.toRadians(-90));
                    drive.setPoseEstimate(new Pose2d(0,0,0));
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(30,0), new ConstantInterpolator(0))
                                    .build()

                    );
                    drive.setPoseEstimate(new Pose2d(0,0,0));
                }


                drive.setPoseEstimate(new Pose2d(0,0,0));

                greenWheelLeftIntake.setPower(1.0);

                greenWheelRightIntake.setPower(-1.0);

                drive.setPoseEstimate(new Pose2d(0,0,0));

                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(-20,0), new ConstantInterpolator(0))
                                .build()
                );




            }

            if(NumberVariable.NUM_OF_STONES.getNumber() > 0){

                if(position == "LEFT"){ }

                else if(position == "CENTER"){}

                else if(position == "RIGHT"){}

                else {}



                if(NumberVariable.NUM_OF_STONES.getNumber() > 0){



                }

            }



    }

}
