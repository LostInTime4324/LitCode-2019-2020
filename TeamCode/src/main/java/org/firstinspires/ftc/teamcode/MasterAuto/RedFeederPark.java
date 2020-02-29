package org.firstinspires.ftc.teamcode.MasterAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.teamcode.BooleanVariable;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.SampleMecanumDriveREVOptimized;


@Autonomous(name="BlueAutoFeeder", group="GodBot")

public class RedFeederPark extends LinearOpMode {

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

        greenWheelRightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        greenWheelLeftIntake = hardwareMap.get(DcMotor.class, "leftIntake");

        while(!isStarted()){

            telemetry.addData(">", "Ready");

        }

        waitForStart();

        //Roadrunner REV Optimized Drive Builder
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(50,0), new ConstantInterpolator(0))
                        .build()
        );

        drive.setPoseEstimate(new Pose2d(0,0,0));

        drive.turnSync(Math.toRadians(90));

        drive.setPoseEstimate(new Pose2d(0,0,0));


        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(10,0), new ConstantInterpolator(0))
                        .build()
        );

        drive.setPoseEstimate(new Pose2d(0,0,0));


        Thread.sleep(20000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(70,0), new ConstantInterpolator(0))
                        .build()
        );

        drive.setPoseEstimate(new Pose2d(0,0,0));

        drive.turnSync(Math.toRadians(90));

        drive.setPoseEstimate(new Pose2d(0,0,0));

        greenWheelLeftIntake.setPower(-1);

        greenWheelRightIntake.setPower(1.0);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(50,0), new ConstantInterpolator(0))
                        .build()
        );

        drive.setPoseEstimate(new Pose2d(0,0,0));

        greenWheelLeftIntake.setPower(0);

        greenWheelRightIntake.setPower(0);

        drive.turnSync(Math.toRadians(90));

        drive.setPoseEstimate(new Pose2d(0,0,0));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(50,0), new ConstantInterpolator(0))
                        .build()
        );

        greenWheelLeftIntake.setPower(1.0);

        greenWheelRightIntake.setPower(-1.0);

        drive.setPoseEstimate(new Pose2d(0,0,0));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(-20,0), new ConstantInterpolator(0))
                        .build()
        );









    }

}
