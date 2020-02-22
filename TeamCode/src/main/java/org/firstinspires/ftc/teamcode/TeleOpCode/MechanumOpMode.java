package org.firstinspires.ftc.teamcode.TeleOpCode;


import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;


@TeleOp(name = "OGScorpionTeleOp", group = "Linear OpMode")

public class MechanumOpMode extends OpMode {



    //Creates empty drive objects
    public static DcMotorEx frontLeftDrive;
    public static DcMotorEx frontRightDrive;
    public static DcMotorEx backLeftDrive;
    public static DcMotorEx backRightDrive;

    //Creates empty elevator objects
    public static DcMotor elevatorMotor;
    public static DcMotor elevatorTilt;

    //Creates empty intake objects
    public static DcMotor greenWheelLeftIntake;
    public static DcMotor greenWheelRightIntake;

    //Creates empty Servo objects
    public static Servo rightHooker;
    public static Servo leftHooker;
    public static Servo clawServo;//h







    @Override
    public void init() {


        //Tells the phone that the robot is initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rightHooker = hardwareMap.get(Servo.class, "rightDropper");
        leftHooker = hardwareMap.get(Servo.class, "leftDropper");

        frontLeftDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "backRightDrive");

        elevatorTilt = hardwareMap.get(DcMotor.class, "elevatorTilter");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator");

        greenWheelRightIntake = hardwareMap.get(DcMotor.class,"rightIntake");
        greenWheelLeftIntake = hardwareMap.get(DcMotor.class,"leftIntake");
//        clawTwist = hardwareMap.get(Servo.class,"clawTwist");

//
        clawServo = hardwareMap.get(Servo.class,"clawServo");
//        clawServo2 = hardwareMap.get(Servo.class,"clawServo2");

        elevatorTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Optimizes direction of motors based on the side of the robot that they are on
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);


        frontLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(STOP_AND_RESET_ENCODER);


        frontLeftDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);

        frontLeftDrive.setVelocityPIDFCoefficients(1.169, .1169, 0, 11.69);

        frontRightDrive.setVelocityPIDFCoefficients(1.129, .1129, 0, 11.29);

        backLeftDrive.setVelocityPIDFCoefficients(1.137, .1137, 0, 11.37);

        backLeftDrive.setVelocityPIDFCoefficients(1.122, .1122, 0, 11.22);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



//        frontLeftDrive.setMode(RUN_WITHOUT_ENCODER);
//        frontRightDrive.setMode(RUN_WITHOUT_ENCODER);
//        backLeftDrive.setMode(RUN_WITHOUT_ENCODER);
//        backRightDrive.setMode(RUN_WITHOUT_ENCODER);







    }
    boolean changed = false, on = false;

    @Override
    public void loop() {



//        leftDrive(gamepad1.left_stick_x);
//        rightDrive(gamepad1.right_stick_x);
//        lateralDrive(gamepad1.left_stick_x);

        telemetry.addData("FrontLeft",frontLeftDrive.getCurrentPosition());
        telemetry.addData("BackLeft",backLeftDrive.getCurrentPosition());
        telemetry.addData("FrontRight", frontRightDrive.getCurrentPosition());
        telemetry.addData("BackRight", backRightDrive.getCurrentPosition());
        telemetry.update();


        telemetry.update();


        drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //Slowed D-pad speed
        if(gamepad1.dpad_up){

            leftDrive(0.4);
            rightDrive(0.4);

        }
        else if (gamepad1.dpad_down){

            leftDrive(-0.4);
            rightDrive(-0.4);

        }
        else if (gamepad1.dpad_left){

            lateralDrive(0.4);

        }
        else if(gamepad1.dpad_right){

            lateralDrive(-0.4);

        }
        else{

            frontRightDrive.setPower(0.0);
            backRightDrive.setPower(0.0);
            frontLeftDrive.setPower(0.0);
            backLeftDrive.setPower(0.0);

        }


        //Runs elevator at slow and fast speeds
        if(gamepad2.left_bumper){

            elevatorTilt.setPower((gamepad2.left_stick_y * 0.5));

        }
        else{

            elevatorTilt.setPower(gamepad2.left_stick_y);


        }

        if(gamepad2.right_bumper){

            elevatorMotor.setPower((gamepad2.right_stick_y * 0.5));

        }
        else{

            elevatorMotor.setPower(gamepad2.right_stick_y);

        }


        //Drops the foundation Hookers
        if(gamepad2.right_stick_button){

            rightHooker.setPosition(0.75);

        }
        else{

            rightHooker.setPosition(0.25);
        }

        if(gamepad2.left_stick_button){

            leftHooker.setPosition(0.3);
        }
        else{

            leftHooker.setPosition(0.90);
        }


        //Runs the claw Servo
        if(gamepad2.x){
            clawServo.setPosition(0.5);

        }
        else{
            clawServo.setPosition(0.65);
        }


        //Toggle example
//




        //Get snagged by dat hooker



//        Paagal Toggle
//        if(gamepad2.b && !changed) {
//
//            daHooker.setPosition(on ? 1 : 0);
//
//            on = !on;
//
//            changed = true;
//
//        }
//
//        else if (!gamepad2.b) {
//
//            changed = false;
//
//        }





//        if(gamepad2.b){
//
//            daHooker.setPosition(NumberVariable.CLAW_END.getNumber());
//
//        }
//        else if(!gamepad2.b){
//
//            daHooker.setPosition(NumberVariable.CLAW_START.getNumber());
//
//        }

//        if(gamepad2.a){
//
//            clawServo2.setPosition(0.65);
//
//        }
//        else if (!gamepad2.a){
//
//            clawServo2.setPosition(0.5);
//
//        }



        //GIves the driver intake controls
        if(gamepad1.right_bumper){

            greenWheelLeftIntake.setPower(1.0);

            greenWheelRightIntake.setPower(-1.0);

        }

        else if(gamepad1.left_bumper){

            greenWheelLeftIntake.setPower(-1.0);

            greenWheelRightIntake.setPower(1.0);

        }
        else{

            greenWheelLeftIntake.setPower(0);

            greenWheelRightIntake.setPower(0);


        }


        //Can be used to bulk read data and debug non-working parts
//        telemetry.addData("Hooker Position", daHooker.getPosition());

//        telemetry.addData("daHooker", gamepad1.y);

//        telemetry.addData("rightDropper", gamepad2.right_stick_button);
//        telemetry.addData("leftDropper", gamepad2.left_stick_button);
//
//
//        telemetry.addData("backLeftPower", gamepad1.left_stick_y);
//        telemetry.addData("frontLeftPower", gamepad1.left_stick_y);
//
//        telemetry.addData("backRightPower", gamepad1.right_stick_y);
//        telemetry.addData("frontRightPower", gamepad1.right_stick_y);
    }




    public void rightDrive (double rightPower){


        if(Math.abs(rightPower) > 0.1){

            frontRightDrive.setPower(rightPower);
            backRightDrive.setPower(rightPower);

        }

        else{

            frontRightDrive.setPower(0.0);
            backRightDrive.setPower(0.0);

        }


    }

    public void leftDrive (double leftPower){


        if(Math.abs(leftPower) > 0.1){
            frontLeftDrive.setPower(leftPower);
            backLeftDrive.setPower(leftPower);

        }
        else{

            frontLeftDrive.setPower(0.0);
            backLeftDrive.setPower(0.0);

        }

    }


    public void lateralDrive(double lateralPower){

        if(Math.abs(lateralPower) > 0.1){

            frontLeftDrive.setPower(-lateralPower);
            backLeftDrive.setPower(lateralPower);
            frontRightDrive.setPower(lateralPower);
            backRightDrive.setPower(-lateralPower);

        }
        else{

            frontLeftDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);

        }

    }

    public void drive(double forward, double strafe, double rotation){

        if(Math.abs(forward) < 0.2){

            forward = 0;

        }
        if(Math.abs(strafe) < 0.2){

            strafe = 0;

        }
        if(Math.abs(rotation) < 0.2){

            rotation = 0;

        }

        backLeftDrive.setPower(forward + strafe - rotation);
        frontLeftDrive.setPower(forward - strafe - rotation);
        backRightDrive.setPower(forward - strafe + rotation);
        frontRightDrive.setPower(forward + strafe + rotation);





    }
}


