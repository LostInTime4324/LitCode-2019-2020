package org.firstinspires.ftc.teamcode.teamcode.TestingStuff.TestCode.BlueAuto;

import android.graphics.*;

import com.qualcomm.hardware.bosch.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.tfod.*;

//Name the Autonomous right here, displayed on driver station dropdown menu

public class OGScorpionAutoBlueRightStoneWaffle extends LinearOpMode {

    //Some lazily declared navigation variables(Jank as heck)
    public double x = 0;
    public double distance = 0;

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

    //Intakes declared
    public static DcMotor greenWheelLeftIntake;
    public static DcMotor greenWheelRightIntake;

    //The elevator motor declared
    public static DcMotor elevatorTilt;


    //Math for the proportional gyro turn and also the encoder drive
    public int countsPerInch = 143;
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    static final double     HEADING_THRESHOLD       = 1 ;

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
        clawServo = hardwareMap.get(Servo.class,"clawServo");

        //All the following are how you name the devices in the configuration
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        greenWheelRightIntake = hardwareMap.get(DcMotor.class,"rightIntake");
        greenWheelLeftIntake = hardwareMap.get(DcMotor.class,"leftIntake");

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
        telemetry.addData(">", "Ready Speagety");
        telemetry.update();


        leftDropper.setPosition(0);
        rightDropper.setPosition(180);


        //Waits for you to push the triangle play button, everything before this happens after init is pushed
        waitForStart();

        elevatorTilt.setPower(1.0);
        Thread.sleep(100);
        elevatorTilt.setPower(0);
        rightDropper.setPosition(0);
        leftDropper.setPosition(180);


        encoderDrive(17,17,17,17,1.0,1.0,1.0,1.0);


        //This does the work to detect the skystone using the color sensor
        while(hsvValues[0] < 107){

            frontLeftDrive.setPower(1.0);
            backLeftDrive.setPower(-1.0);
            frontRightDrive.setPower(-1.0);
            backRightDrive.setPower(1.0);

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR), (int) (sensorColor.green() * SCALE_FACTOR), (int) (sensorColor.blue() * SCALE_FACTOR), hsvValues);

            telemetry.addData("Stone", "Not detected");

            x++;

            telemetry.addData("x-value", x);

            telemetry.addData("hsv", hsvValues[0] );

            telemetry.update();
        }


        frontLeftDrive.setPower(-1.0);
        backLeftDrive.setPower(1.0);
        frontRightDrive.setPower(1.0);
        backRightDrive.setPower(-1.0);

        Thread.sleep(500);

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);


        if(x<9){

            distance = 56;

        }
        else if(x>=9 && x<16){

            distance = 61;

        }

        else if(x>=16){

            distance = 66;

        }
        else{

            distance = 66;

        }

        telemetry.addData("Stone", "Detected");
        telemetry.addData("x-value", x);
        telemetry.update();


        greenWheelLeftIntake.setPower(0.7);
        greenWheelRightIntake.setPower(0.7);


        frontLeftDrive.setPower(1.0);
        backLeftDrive.setPower(1.0);
        frontRightDrive.setPower(1.0);
        backRightDrive.setPower(1.0);
        Thread.sleep(700);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        Thread.sleep(800);
        frontLeftDrive.setPower(-1.0);
        backLeftDrive.setPower(-1.0);
        frontRightDrive.setPower(-1.0);
        backRightDrive.setPower(-1.0);
        Thread.sleep(700);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);


        clawServo.setPosition(0.5);
        elevatorTilt.setPower(-1.0);
        Thread.sleep(200);
        elevatorTilt.setPower(0);


        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        greenWheelLeftIntake.setPower(0);
        greenWheelRightIntake.setPower(0);


        gyroTurn(0.5,85);
        encoderDrive(distance,distance,distance,distance,1.0,1.0,1.0,1.0);
        gyroTurn(1.0, 180);


        elevatorTilt.setPower(-1.0);
        Thread.sleep(200);
        elevatorTilt.setPower(0);
        clawServo.setPosition(0.65);
        Thread.sleep(100);
        elevatorTilt.setPower(1.0);
        Thread.sleep(1800);
        elevatorTilt.setPower(0);
        clawServo.setPosition(0.5);


        elevatorTilt.setPower(-1.0);
        gyroTurn(0.7,90);
        elevatorTilt.setPower(0);
        frontLeftDrive.setPower(1.0);
        backLeftDrive.setPower(-1.0);
        frontRightDrive.setPower(-1.0);
        backRightDrive.setPower(1.0);
        Thread.sleep(1000);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        elevatorTilt.setPower(0);


        daHooker.setPosition(1.0);
        Thread.sleep(400);
        frontLeftDrive.setPower(-1.0);
        backLeftDrive.setPower(1.0);
        frontRightDrive.setPower(1.0);
        backRightDrive.setPower(-1.0);
        Thread.sleep(3500);
        Thread.sleep(200);
        gyroTurn(0.7,180);
        daHooker.setPosition(0);



        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        encoderDrive(-12,-12,-12,-12,-1.0,-1.0,-1.0,-1.0);

        frontLeftDrive.setPower(-1.0);
        backLeftDrive.setPower(1.0);
        frontRightDrive.setPower(1.0);
        backRightDrive.setPower(-1.0);

        Thread.sleep(3000);

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

    }





    //METHODS AND SPECIAL VARIABLES HERE


    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getCorrectedHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }



    float getCorrectedHeading(){
        //Initialize our gyro
        Orientation gyro = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        float uncorrectedHeading = gyro.thirdAngle;
        float correctedHeading = 0;
        //Do some math to correct the heading values
        if (uncorrectedHeading > 0){

            correctedHeading = gyro.thirdAngle;

        } else if(uncorrectedHeading < 0){

            correctedHeading = 180 + (180 - Math.abs(gyro.thirdAngle));

        }
        //Returns corrected heading value
        return correctedHeading;
    }



//    private void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
//    }



    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }





    public void gyroDrive ( double speed, double distance, double angle) {

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
            }}}


    public void gyroHold ( double gyroSpeed, double gyroAngle, double holdTime){

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


    public double getSteer ( double error, double PCoeff){
        return Range.clip(error * PCoeff, -1, 1);
    }


    boolean onHeading ( double speed, double angle, double PCoeff){
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


//            private void initTfod () {
//                int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                        "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//                TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//                tfodParameters.minimumConfidence = 0.8;
//                tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//                tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
//            }


    void encoderDrive ( double frontLeftInches, double backLeftInches, double frontRightInches, double backRightInches, double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed){

        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

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


        frontRightDrive.setPower(frontRightSpeed);
        backRightDrive.setPower(backRightSpeed);
        frontLeftDrive.setPower(frontLeftSpeed);
        backLeftDrive.setPower(backLeftSpeed);


        while (backLeftDrive.isBusy() || frontLeftDrive.isBusy() || backRightDrive.isBusy() || frontRightDrive.isBusy()) {

            telemetry.addData("targetPosition", String.valueOf(newBackLeftTarget), String.valueOf(newFrontLeftTarget), String.valueOf(newBackRightTarget), String.valueOf(newFrontRightTarget));
            telemetry.addData("currentPosition", String.valueOf(backLeftDrive.getCurrentPosition()), String.valueOf(frontLeftDrive.getCurrentPosition()), String.valueOf(backRightDrive.getCurrentPosition()), String.valueOf(frontRightDrive.getCurrentPosition()));
            telemetry.update();

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
