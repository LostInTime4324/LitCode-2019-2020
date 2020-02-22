package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;


@TeleOp(name = "SpeakTest", group = "Linear OpMode")


public class SpeakTest extends OpMode {


    @Override
    public void init() {

        telemetry.addData(">", "Speaking Now");
        telemetry.speak("Roger cowboys, The skystone haaaas been detected", "aar", "AE");
        telemetry.update();



    }

    @Override
    public void loop() {


    }
}
