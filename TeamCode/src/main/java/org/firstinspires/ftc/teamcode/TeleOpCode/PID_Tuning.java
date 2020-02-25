package org.firstinspires.ftc.teamcode.TeleOpCode;

import com.acmerobotics.dashboard.config.*;
import com.acmerobotics.roadrunner.drive.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;


@Config
@Autonomous(group = "PID_Tuning")
public class PID_Tuning extends LinearOpMode{
    DcMotorEx frontRightDrive;
//frontright 2820

        DcMotorEx frontLeftDrive;
// frontleft 2900
        DcMotorEx backRightDrive;
    // 2880
        DcMotorEx backLeftDrive;
    // 2920
        double currentVelocityFrontLeft;
        double maxVelocityFrontLeft = 0.0;

    double currentVelocityFrontRight;
    double maxVelocityFrontRight = 0.0;

    double currentVelocityBackRight;
    double maxVelocityBackRight = 0.0;

    double currentVelocityBackLeft;
    double maxVelocityBackLeft = 0.0;

        @Override
        public void runOpMode() {
            frontRightDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "frontRightDrive");
            frontLeftDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "frontLeftDrive");
            backRightDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "backRightDrive");
            backLeftDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "backLeftDrive");

            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

;
            waitForStart();

            while (opModeIsActive()) {
                backLeftDrive.setPower(1.0);
                currentVelocityBackLeft = backLeftDrive.getVelocity();

                if (currentVelocityBackLeft > maxVelocityBackLeft) {
                    maxVelocityBackLeft = currentVelocityBackLeft;
                }

                backRightDrive.setPower(1.0);
                currentVelocityBackRight = backRightDrive.getVelocity();

                if (currentVelocityBackRight > maxVelocityBackRight) {
                    maxVelocityBackRight = currentVelocityBackRight;
                }

                frontLeftDrive.setPower(1.0);
                currentVelocityFrontLeft = frontLeftDrive.getVelocity();

                if (currentVelocityFrontLeft > maxVelocityFrontLeft) {
                    maxVelocityFrontLeft = currentVelocityFrontLeft;
                }

                frontRightDrive.setPower(1.0);
                currentVelocityFrontRight = frontRightDrive.getVelocity();

                if (currentVelocityFrontRight > maxVelocityFrontRight) {
                    maxVelocityFrontRight = currentVelocityFrontRight;
                }





                telemetry.addData("current velocityBL", currentVelocityBackLeft);
                telemetry.addData("maximum velocityBL", maxVelocityBackLeft);

                telemetry.addData("current velocityBR", currentVelocityBackRight);
                telemetry.addData("maximum velocityBR", maxVelocityBackRight);

                telemetry.addData("current velocityFL", currentVelocityFrontLeft);
                telemetry.addData("maximum velocityFL", maxVelocityFrontLeft);

                telemetry.addData("current velocityFR", currentVelocityFrontRight);
                telemetry.addData("maximum velocityFR", maxVelocityFrontRight);
                telemetry.update();
            }
        }
    }




