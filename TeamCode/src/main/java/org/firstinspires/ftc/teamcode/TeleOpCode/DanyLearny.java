package org.firstinspires.ftc.teamcode.TeleOpCode;


import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;


public class DanyLearny extends OpMode {
    public DcMotor DanyKart;

    @Override
    public void init() {

        DanyKart = hardwareMap.get(DcMotor.class,"Dany's Kart");
    }

    @Override
    public void loop() {

        DanyKart.setPower(gamepad1.right_trigger * 1.0);
        DanyKart.setPower(gamepad1.left_trigger * -1.0);

    }
    public static void main(String[] args) {
        System.out.println("Lets see if Git will work!");

    }
}

