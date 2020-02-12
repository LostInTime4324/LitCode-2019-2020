package org.firstinspires.ftc.teamcode.teamcode.OfficialAuto.Blueside.OneStone;

import android.graphics.*;

import com.qualcomm.hardware.bosch.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcontroller.teamcode.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.tfod.*;

//@Autonomous(name="BlueAutoStoneOnly", group="Pushbot")

public class BlueAutoStoneOnly extends LinearOpMode {

    //Some lazily declared navigation variables(Jank as heck)
    public double x = 0;
    public double distance = 0;
    boolean redSide = false;
    boolean blueSide = false;
    boolean waffleSide = false;
    boolean skystoneSide = false;
    boolean doWaffle = false;
    boolean parkOnTop = false;
    boolean parkOnBottom = false;

    //These declare the color/distance sensor objects that are later named in the hardware map
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    //Gyro variables
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Drive motors are declared
    public static DcMotor frontLeftDrive;
    public static DcMotor frontRightDrive;
    public static DcMotor backLeftDrive;
    public static DcMotor backRightDrive;

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

    DigitalChannel TouchyTouchy;


    //Math for the proportional gyro turn and also the encoder drive
    public int countsPerInch = 143;
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 1;

    //Tensor flow object detection variables
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AVZA5qD/////AAABmRGmibKwykLGhTqM/VfCJ+84E/FI84Y5XqFPxvsEEihjanY1la2EAGQJ4uDQ7KNit2kOxS/tGvpxDvQy4qppcC0TXRMGBOX4boSsf/G7g5KvJB2YTFRKxHT+9AIzghvGBv3POdbINbGDAUMJw0Xl/muf//0g6PYYV9FRd3Kk7xo/XTC45am3l/hnSWFnUiA9jy7tZNeWb6wlI7E4FN69jRA1d/ROfHmrCY/akzB0trQBVMCk2SJPqII2RVTqNJyhyWXX564dsFg6o2E68+PkcRq/1ggHcALpa84peMvXjBvSmUA75DQhHqu7xXIusBYRxBUFGS1OwaBoQHjn62ZDCWdN1yU32ZdqRtzhbhIuVGae";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Tell the rev hub which motors are plugged in where
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        elevatorTilt = hardwareMap.get(DcMotor.class, "elevatorTilter");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo2 = hardwareMap.get(Servo.class,"clawServo2");


        TouchyTouchy = hardwareMap.get(DigitalChannel.class, "TouchyTouchy");

        //All the following are how you name the devices in the configuration
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        greenWheelRightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        greenWheelLeftIntake = hardwareMap.get(DcMotor.class, "leftIntake");

        daHooker = hardwareMap.get(Servo.class, "daHooker");

        rightDropper = hardwareMap.get(Servo.class, "rightDropper");
        leftDropper = hardwareMap.get(Servo.class, "leftDropper");


        //Reset the encoders
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set the motors to run using encoders
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reverse the motors because they are flipped on each side of the robot
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //Brake the elevator so it doesn't slowly tilt down
        elevatorTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TouchyTouchy.setMode(DigitalChannel.Mode.INPUT);


        //
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;


        //This stuff is tensor flow activation
        // initVuforia();

//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        //        if (tfod != null) {
        //            tfod.activate();
        //        }

        /** Wait for the game to begin */




        leftDropper.setPosition(0);
        rightDropper.setPosition(180);

        telemetry.addData(">","Scorpion Bot is Locked and Loaded Baby");
        telemetry.update();

        //Waits for you to push the triangle play button, everything before this happens after init is pushed
        waitForStart();
        elevatorTilt.setPower(1.0);
        Thread.sleep(100);
        elevatorTilt.setPower(0);
        rightDropper.setPosition(0);
        leftDropper.setPosition(180);

//
//        encoderDrive(NumberVariable.FIRST_DRIVE.getNumber(), NumberVariable.FIRST_DRIVE.getNumber(), NumberVariable.FIRST_DRIVE.getNumber(), NumberVariable.FIRST_DRIVE.getNumber(), 0.8, 0.8, 0.8, 0.8);
//
//        //This does the work to detect the skystone using the color sensor
//        while (hsvValues[0] < 99) {
//
//            frontLeftDrive.setPower(NumberVariable.SLIDE_SPEED.getNumber());
//            backLeftDrive.setPower(-NumberVariable.SLIDE_SPEED.getNumber());
//            frontRightDrive.setPower(-NumberVariable.SLIDE_SPEED.getNumber());
//            backRightDrive.setPower(NumberVariable.SLIDE_SPEED.getNumber());
//
//            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR), (int) (sensorColor.green() * SCALE_FACTOR), (int) (sensorColor.blue() * SCALE_FACTOR), hsvValues);
//
//            telemetry.addData("Stone", "Not detected");
//
//            x++;
//
//            telemetry.addData("x-value", x);
//
//            telemetry.addData("hsv", hsvValues[0]);
//
//            telemetry.update();
//        }
//
//        frontLeftDrive.setPower(0);
//        backLeftDrive.setPower(0);
//        frontRightDrive.setPower(0);
//        backRightDrive.setPower(0);

//        if (x < 12) {
//
//            distance = NumberVariable.FIRST_DISTANCE_TO_WAFFLE.getNumber()-15;
//
//        } else if (x >= 12 && x < 50) {
//
//            distance = NumberVariable.SECOND_DISTANCE_TO_WAFFLE.getNumber()-15;
//
//        } else if (x >= 50) {
//
//            distance = NumberVariable.THIRD_DISTANCE_TO_WAFFLE.getNumber()-15;
//
//        } else {
//
//            distance = NumberVariable.THIRD_DISTANCE_TO_WAFFLE.getNumber()-15;
//
//        }
//
////        encoderDrive( NumberVariable.SLIDE_CORRECTION.getNumber(), - NumberVariable.SLIDE_CORRECTION.getNumber(), - NumberVariable.SLIDE_CORRECTION.getNumber(),  NumberVariable.SLIDE_CORRECTION.getNumber(),  0.5, - 0.5, - 0.5,  0.5);
//
//        gyroTurn(0.5, NumberVariable.FIRST_TURN.getNumber());
//
//        greenWheelLeftIntake.setPower(1.0);
//        greenWheelRightIntake.setPower(-1.0);
//
//        encoderDrive(NumberVariable.DRIVE_TO_STONE.getNumber(), NumberVariable.DRIVE_TO_STONE.getNumber(), NumberVariable.DRIVE_TO_STONE.getNumber(), NumberVariable.DRIVE_TO_STONE.getNumber(), 0.5, 0.5, 0.5, 0.5);
//
//        encoderDrive(-(NumberVariable.DRIVE_TO_STONE.getNumber()-6), -(NumberVariable.DRIVE_TO_STONE.getNumber()-6), -(NumberVariable.DRIVE_TO_STONE.getNumber()-6), -(NumberVariable.DRIVE_TO_STONE.getNumber()-6), -1.0, -1.0, -1.0, -1.0);
//
//        greenWheelLeftIntake.setPower(0);
//        greenWheelRightIntake.setPower(0);
//
//        gyroTurn(0.5, NumberVariable.TURN_TO_WAFFLE.getNumber());

        encoderDrive( distance,  distance,  distance,  distance,  1.0,  1.0,  1.0,  1.0);

        clawServo2.setPosition(0.65);

        greenWheelLeftIntake.setPower(-1.0);
        greenWheelRightIntake.setPower(1.0);

        Thread.sleep(500);

        encoderDrive( (-distance/2)+3,  (-distance/2)+3,  (-distance/2)+3,  (-distance/2)+3,  -1.0,  -1.0,  -1.0,  -1.0);




    }


    //METHODS AND SPECIAL VARIABLES HERE


    //Gets correct heading from gyro values in a better more readable way
    private float getCorrectedHeading() {
        //Initialize our gyro
        Orientation gyro = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        float uncorrectedHeading = gyro.thirdAngle;
        float correctedHeading = 0;
        //Do some math to correct the heading values
        if (uncorrectedHeading > 0) {

            correctedHeading = gyro.thirdAngle;

        } else if (uncorrectedHeading < 0) {

            correctedHeading = 180 + (180 - Math.abs(gyro.thirdAngle));

        }
        //Returns corrected heading value
        return correctedHeading;
    }

    //Gets the error with desired and current heading
    private double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getCorrectedHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    //Gets the proportional amount of change we want to make based on error
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //Checks whether we are on the desired heading, and moves if we aren't
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        frontLeftDrive.setPower(leftSpeed);
        backLeftDrive.setPower(leftSpeed);
        frontRightDrive.setPower(rightSpeed);
        backRightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    //Sets where we want to turn and what speed using all the previous methods
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    //Drives while maintaining a certain heading
    private void gyroDrive(double speed, double distance, double angle) {

        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * countsPerInch);
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + moveCounts;
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
            newBackRightTarget = backRightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            frontLeftDrive.setPower(speed);
            backLeftDrive.setPower(speed);
            frontRightDrive.setPower(speed);
            backRightDrive.setPower(speed);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backLeftDrive.isBusy() && backRightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                frontLeftDrive.setPower(leftSpeed);
                backLeftDrive.setPower(leftSpeed);
                frontRightDrive.setPower(rightSpeed);
                backRightDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Actual", "%7d:%7d", frontLeftDrive.getCurrentPosition(),
                        frontRightDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }
        }
    }

    //Just stands there looking one way, if ya try to move the bot it'll turn back mate
    public void gyroHold(double gyroSpeed, double gyroAngle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(gyroSpeed, gyroAngle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    //Drives based on encoder values using PID in order to maintain velocity
    void encoderDrive(double frontLeftInches, double backLeftInches, double frontRightInches, double backRightInches, double frontLeftSpeed, double backLeftSpeed, double frontRightSpeed, double backRightSpeed) {

        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (frontLeftInches * countsPerInch);

        newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int) (backLeftInches * countsPerInch);

        newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) (backRightInches * countsPerInch);

        newBackRightTarget = backRightDrive.getCurrentPosition() + (int) (frontRightInches * countsPerInch);

        backLeftDrive.setTargetPosition(newBackLeftTarget);
        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
        backRightDrive.setTargetPosition(newFrontRightTarget);
        frontRightDrive.setTargetPosition(newBackRightTarget);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        while (Math.abs(frontLeftDrive.getCurrentPosition()) < (Math.abs(newFrontLeftTarget) - 60) || Math.abs(frontRightDrive.getCurrentPosition()) < (Math.abs(newFrontRightTarget) - 60) || Math.abs(backLeftDrive.getCurrentPosition()) < (Math.abs(newBackLeftTarget) - 60) || Math.abs(backRightDrive.getCurrentPosition()) < (Math.abs(newBackRightTarget) - 60) ) {

            double frontLeftError;
            double backLeftError;
            double frontRightError;
            double backRightError;

            telemetry.addData("targetPosition", String.valueOf(newBackLeftTarget), String.valueOf(newFrontLeftTarget), String.valueOf(newBackRightTarget), String.valueOf(newFrontRightTarget));
            telemetry.addData("currentPosition", String.valueOf(backLeftDrive.getCurrentPosition()), String.valueOf(frontLeftDrive.getCurrentPosition()), String.valueOf(backRightDrive.getCurrentPosition()), String.valueOf(frontRightDrive.getCurrentPosition()));
            telemetry.update();

            frontLeftError = Math.abs(newFrontLeftTarget) - Math.abs(frontLeftDrive.getCurrentPosition());
            backLeftError = Math.abs(newBackLeftTarget) - Math.abs(backLeftDrive.getCurrentPosition());
            frontRightError = Math.abs(newFrontRightTarget) - Math.abs(frontRightDrive.getCurrentPosition());
            backRightError = Math.abs(newBackRightTarget) - Math.abs(backRightDrive.getCurrentPosition());

            if(frontRightSpeed > 0){

                frontRightDrive.setPower(Range.clip((frontRightSpeed * (frontRightError * P_DRIVE_COEFF)), -frontRightSpeed, frontRightSpeed));

            }
            else if(frontRightSpeed < 0){

                frontRightDrive.setPower(Range.clip((frontRightSpeed * (frontRightError * P_DRIVE_COEFF)), frontRightSpeed, -frontRightSpeed));

            }

            if(backRightSpeed > 0){

                backRightDrive.setPower(Range.clip((backRightSpeed * (backRightError * P_DRIVE_COEFF)), -backRightSpeed, backRightSpeed));

            }
            else if(backRightSpeed < 0){

                backRightDrive.setPower(Range.clip((backRightSpeed * (backRightError * P_DRIVE_COEFF)), backRightSpeed, -backRightSpeed));

            }

            if(frontLeftSpeed > 0){

                frontLeftDrive.setPower(Range.clip((frontLeftSpeed * (frontLeftError * P_DRIVE_COEFF)), -frontLeftSpeed, frontLeftSpeed));

            }
            else if(frontLeftSpeed < 0){

                frontLeftDrive.setPower(Range.clip((frontLeftSpeed * (frontLeftError * P_DRIVE_COEFF)), frontLeftSpeed, -frontLeftSpeed));

            }

            if(backLeftSpeed > 0){

                backLeftDrive.setPower(Range.clip((backLeftSpeed * (backLeftError * P_DRIVE_COEFF)), -backLeftSpeed, backLeftSpeed));

            }
            else if(backLeftSpeed < 0){

                backLeftDrive.setPower(Range.clip((backLeftSpeed * (backLeftError * P_DRIVE_COEFF)), backLeftSpeed, -backLeftSpeed));

            }

        }


        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    void RampedEncoderDrive(double rampCoefficient, double frontLeftInches, double backLeftInches, double frontRightInches, double backRightInches, double frontLeftSpeed, double backLeftSpeed, double frontRightSpeed, double backRightSpeed) {

        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        double currentFrontLeftSpeed = 0;
        double currentFrontRightSpeed = 0;
        double currentBackLeftSpeed = 0;
        double currentBackRightSpeed = 0;

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (frontLeftInches * countsPerInch);

        newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int) (backLeftInches * countsPerInch);

        newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) (backRightInches * countsPerInch);

        newBackRightTarget = backRightDrive.getCurrentPosition() + (int) (frontRightInches * countsPerInch);

        backLeftDrive.setTargetPosition(newBackLeftTarget);
        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
        backRightDrive.setTargetPosition(newFrontRightTarget);
        frontRightDrive.setTargetPosition(newBackRightTarget);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (Math.abs(frontLeftDrive.getCurrentPosition()) < (Math.abs(newFrontLeftTarget) - 10) || Math.abs(frontRightDrive.getCurrentPosition()) < (Math.abs(newFrontRightTarget) - 10) || backLeftDrive.getCurrentPosition() < (newBackLeftTarget - 10) || backRightDrive.getCurrentPosition() < (newBackRightTarget - 10)) {

            telemetry.addData("targetPosition", String.valueOf(newBackLeftTarget), String.valueOf(newFrontLeftTarget), String.valueOf(newBackRightTarget), String.valueOf(newFrontRightTarget));
            telemetry.addData("currentPosition", String.valueOf(backLeftDrive.getCurrentPosition()), String.valueOf(frontLeftDrive.getCurrentPosition()), String.valueOf(backRightDrive.getCurrentPosition()), String.valueOf(frontRightDrive.getCurrentPosition()));
            telemetry.addData("currentSpeed", currentFrontRightSpeed);
            telemetry.addData("targetSpeed", frontRightSpeed);
            telemetry.update();
            while (currentFrontLeftSpeed < Math.abs(frontLeftSpeed) || currentBackLeftSpeed < Math.abs(backLeftSpeed) || currentFrontRightSpeed < Math.abs(frontRightSpeed) || currentBackRightSpeed < Math.abs(backRightSpeed) || (Math.abs(frontLeftDrive.getCurrentPosition()) < Math.abs(newFrontLeftTarget / 4))) {

                double frontLeftRamp = (frontLeftSpeed / -frontLeftSpeed) * rampCoefficient;
                double backleftRamp = (backLeftSpeed / -backLeftSpeed) * rampCoefficient;
                double frontRightRamp = (frontRightSpeed / -frontRightSpeed) * rampCoefficient;
                double backRightRamp = (backRightSpeed / -backRightSpeed) * rampCoefficient;

                currentFrontLeftSpeed = currentFrontLeftSpeed - frontLeftRamp;
                currentBackLeftSpeed = currentBackLeftSpeed - backleftRamp;
                currentFrontRightSpeed = currentFrontRightSpeed - frontRightRamp;
                currentBackRightSpeed = currentBackRightSpeed - backRightRamp;

                frontLeftDrive.setPower(currentFrontLeftSpeed);
                backLeftDrive.setPower(currentBackLeftSpeed);
                frontRightDrive.setPower(currentFrontRightSpeed);
                backRightDrive.setPower(currentBackRightSpeed);
            }

            frontRightDrive.setPower(frontRightSpeed);
            backRightDrive.setPower(backRightSpeed);
            frontLeftDrive.setPower(frontLeftSpeed);
            backLeftDrive.setPower(backLeftSpeed);

            if ((Math.abs(frontLeftDrive.getCurrentPosition()) < Math.abs(newFrontLeftTarget * 0.75))) {

                while (currentFrontLeftSpeed > 0 || currentBackLeftSpeed > 0 || currentFrontRightSpeed > 0 || currentBackRightSpeed > 0) {

                    double frontLeftSpeedRampDown = (frontLeftSpeed / -frontLeftSpeed) * ((newFrontLeftTarget - frontLeftDrive.getCurrentPosition()) / (newFrontLeftTarget * 0.75));
                    double backLeftSpeedRampDown = (backLeftSpeed / -backLeftSpeed) * ((newBackLeftTarget - backLeftDrive.getCurrentPosition()) / (newBackLeftTarget * 0.75));
                    double frontRightSpeedRampDown = (frontRightSpeed / -frontRightSpeed) * ((newFrontRightTarget - frontRightDrive.getCurrentPosition()) / (newFrontRightTarget * 0.75));
                    double backRightSpeedRampDown = (backRightSpeed / -backRightSpeed) * ((newBackRightTarget - backRightDrive.getCurrentPosition()) / (newBackRightTarget * 0.75));

                    currentFrontLeftSpeed = currentFrontLeftSpeed - frontLeftSpeedRampDown;
                    currentBackLeftSpeed = currentBackLeftSpeed - backLeftSpeedRampDown;
                    currentFrontRightSpeed = currentFrontRightSpeed - frontRightSpeedRampDown;
                    currentBackRightSpeed = currentBackRightSpeed - backRightSpeedRampDown;

                    frontLeftDrive.setPower(currentFrontLeftSpeed);
                    backLeftDrive.setPower(currentBackLeftSpeed);
                    frontRightDrive.setPower(currentFrontRightSpeed);
                    backRightDrive.setPower(currentBackRightSpeed);


                }

            }


        }


        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


}
