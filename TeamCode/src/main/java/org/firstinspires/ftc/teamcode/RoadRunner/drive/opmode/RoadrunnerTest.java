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



        DriveConstraints constants = new DriveConstraints(40,40,40,40,40,40);

        Path path = new PathBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Pose2d(15, 15, Math.toRadians(180)))
                .lineTo(new Vector2d(0, 15), new ConstantInterpolator(0))
                .build();

        Trajectory Strafe = TrajectoryGenerator.INSTANCE.generateTrajectory(path, constants);



        waitForStart();
//                .strafeLeft(70)
//                .strafeLeft(-70)

        if (isStopRequested()) return;



// call in loop

        PIDCoefficients translationalPid = new PIDCoefficients(0.5, 0, 0.01);
        PIDCoefficients headingPid = new PIDCoefficients(5, 0.2, 0.1);
        HolonomicPIDVAFollower follower = new HolonomicPIDVAFollower(translationalPid, translationalPid, headingPid);

        follower.followTrajectory(Strafe);

        DriveSignal signal = follower.update(drive.getPoseEstimate());


    }
}
