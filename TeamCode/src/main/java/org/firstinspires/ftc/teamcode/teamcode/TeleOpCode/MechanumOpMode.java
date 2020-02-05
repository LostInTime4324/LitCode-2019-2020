package org.firstinspires.ftc.teamcode.teamcode.TeleOpCode;


import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcontroller.teamcode.*;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;


@TeleOp(name = "OGScorpionTeleOp", group = "Linear OpMode")

public class MechanumOpMode extends OpMode {



    //Tells the robot the names of the motors
    public static DcMotor frontLeftDrive;
    public static DcMotor frontRightDrive;
    public static DcMotor backLeftDrive;
    public static DcMotor backRightDrive;
//    public static DcMotor elevatorMotor;
    public static DcMotor elevatorTilt;
    public static DcMotor greenWheelLeftIntake;
    public static DcMotor greenWheelRightIntake;
    public static Servo rightDropper;
    public static Servo leftDropper;
    public static Servo daHooker;
    public static Servo clawServo;
    public static Servo clawServo2;
//    public static Servo clawTwist;





    @Override
    public void init() {

        //Tells the phone that the robot is initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        daHooker = hardwareMap.get(Servo.class, "daHooker");

        rightDropper = hardwareMap.get(Servo.class, "rightDropper");
        leftDropper = hardwareMap.get(Servo.class, "leftDropper");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        elevatorTilt = hardwareMap.get(DcMotor.class, "elevatorTilter");
//        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator");

        greenWheelRightIntake = hardwareMap.get(DcMotor.class,"rightIntake");
        greenWheelLeftIntake = hardwareMap.get(DcMotor.class,"leftIntake");
//        clawTwist = hardwareMap.get(Servo.class,"clawTwist");
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        clawServo2 = hardwareMap.get(Servo.class,"clawServo2");

        elevatorTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Optimizes direction of motors based on the side of the robot that they are on
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        frontLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(STOP_AND_RESET_ENCODER);


        frontLeftDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);

        elevatorTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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

        if(!gamepad1.right_bumper){
            drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        else if (gamepad1.right_bumper){

            frontRightDrive.setPower(gamepad1.left_stick_x);
            backRightDrive.setPower(-gamepad1.left_stick_x);
            frontLeftDrive.setPower(-gamepad1.left_stick_x);
            backLeftDrive.setPower(gamepad1.left_stick_x);

        }
        else if(gamepad1.left_bumper){

            drive(gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);

        }
        else if(gamepad1.left_bumper && gamepad1.right_bumper){

            frontRightDrive.setPower(gamepad1.left_stick_x * 0.5);
            backRightDrive.setPower(-gamepad1.left_stick_x * 0.5);
            frontLeftDrive.setPower(-gamepad1.left_stick_x * 0.5);
            backLeftDrive.setPower(gamepad1.left_stick_x * 0.5);

        }

//        boolean tank = false;
//
//        if(gamepad1.start){
//            tank = true;
//        }
//
//        if(tank == true){
//            if(gamepad1.a){
//                frontLeftDrive.setPower(-gamepad1.left_stick_x);
//                backLeftDrive.setPower(gamepad1.left_stick_x);
//                frontRightDrive.setPower(gamepad1.left_stick_x);
//                backRightDrive.setPower(-gamepad1.left_stick_x);
//
//            }
//
//            else{
//
//                backLeftDrive.setPower(gamepad1.left_stick_y);
//                frontLeftDrive.setPower(gamepad1.left_stick_y);
//                backRightDrive.setPower(gamepad1.right_stick_y);
//                frontRightDrive.setPower(gamepad1.right_stick_y);
//
//
//            }
//
//        }

//
//


//        //test code for one joystick 100% movement



//        frontRightDrive.setPower(-gamepad1.right_stick_x);
//        frontLeftDrive.setPower(gamepad1.right_stick_x);
//        backRightDrive.setPower(-gamepad1.right_stick_x);
//        backLeftDrive.setPower(gamepad1.right_stick_x);
//

        //turning right stick






//        frontRightDrive.setPower(1.0);
//        backRightDrive.setPower(1.0);
//        frontLeftDrive.setPower(1.0);
//        backLeftDrive.setPower(1.0);


        elevatorTilt.setPower(gamepad2.left_stick_y);




        if(gamepad2.right_stick_button){

            rightDropper.setPosition(0.75);

        }
        else{

            rightDropper.setPosition(0.25);
        }

        if(gamepad2.left_stick_button){

            leftDropper.setPosition(0.3);
        }
        else{

            leftDropper.setPosition(0.90);
        }







        if(gamepad2.x){
            clawServo.setPosition(0.5);

        }
        else{
            clawServo.setPosition(0.65);
        }





//        if(gamepad2.y){
//            elevatorMotor.setPower(1.0);
//        }
//        else if(gamepad2.b && gamepad2.y) {
//            elevatorMotor.setPower(-0.2);
//        }
//
//
//
//
//
//
//        if(gamepad2.left_stick_y > 0.1){
//            clawTwist.setPosition(gamepad2.left_stick_y);
//        }
//        else{
//            clawTwist.setPosition(0.5);
//        }

        //Get snagged by dat hooker



//        Paagal Toggle
        if(gamepad2.b && !changed) {

            daHooker.setPosition(on ? 1 : 0);

            on = !on;

            changed = true;

        }

        else if (!gamepad2.b) {

            changed = false;

        }





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

        if(gamepad2.a){

            clawServo2.setPosition(0.65);

        }
        else if (!gamepad2.a){

            clawServo2.setPosition(0.5);

        }




        if(gamepad2.right_bumper){

            greenWheelLeftIntake.setPower(1.0);

            greenWheelRightIntake.setPower(-1.0);

        }

        else if(gamepad2.left_bumper){

            greenWheelLeftIntake.setPower(-1.0);

            greenWheelRightIntake.setPower(1.0);

        }
        else{

            greenWheelLeftIntake.setPower(0);

            greenWheelRightIntake.setPower(0);


        }


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


