package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.dashboard.config.*;
import com.acmerobotics.roadrunner.control.*;
import com.acmerobotics.roadrunner.drive.*;
import com.acmerobotics.roadrunner.followers.*;
import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.path.*;
import com.acmerobotics.roadrunner.path.heading.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.*;

@Config
@Autonomous(group = "RoadrunnerTest")
public class RoadrunnerTest extends LinearOpMode {
    public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        Trajectory strafeLeft = drive.trajectoryBuilder()
                .lineTo(new Vector2d(30,0), new ConstantInterpolator(0))
                .build();

        Trajectory goBack = drive.trajectoryBuilder()
                .back(30)
                .build();

        Trajectory goForward = drive.trajectoryBuilder()
                .forward(30)
                .build();

        waitForStart();


        if (isStopRequested()) return;

        drive.followTrajectorySync(strafeLeft);
        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.turnSync(Math.toRadians(180));
        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.followTrajectorySync(strafeLeft);
        drive.followTrajectorySync(goForward);
        drive.turnSync(Math.toRadians(90));
        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.followTrajectorySync(goForward);
        drive.turnSync(Math.toRadians(90));
        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.followTrajectorySync(goForward);
        drive.turnSync(Math.toRadians(90));
        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.followTrajectorySync(goForward);
        drive.turnSync(Math.toRadians(90));

        telemetry.addData(">", "Roadrunner instantiated");
        telemetry.update();








// call in loop



    }
}
