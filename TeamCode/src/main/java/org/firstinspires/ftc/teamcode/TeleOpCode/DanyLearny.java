package org.firstinspires.ftc.teamcode.TeleOpCode;


import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

public class DanyLearny extends OpMode {

    public DcMotor Turny;



    @Override
    public void init() {

        Turny = hardwareMap.get(DcMotor.class,"UwU");



    }

    @Override
    public void loop() {


        Turny.setPower(gamepad1.left_stick_y);

    }


}
