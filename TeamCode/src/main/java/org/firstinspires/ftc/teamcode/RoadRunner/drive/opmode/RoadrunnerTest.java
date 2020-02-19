package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.dashboard.config.*;
import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.path.heading.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.*;

@Config
@Autonomous(group = "RoadrunnerTest")
public class RoadrunnerTest extends LinearOpMode {
    public static double DISTANCE = 60;




    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
//                .lineTo(new Vector2d(10, -60), new LinearInterpolator(0, Math.toRadians(-20)))
                .strafeLeft(30)
//                .splineTo(new Pose2d(30,-140),new LinearInterpolator(0,Math.toRadians(180)))
//                .splineTo(new Pose2d(0,0),new LinearInterpolator(Math.toRadians(180), 0))

                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }
}
