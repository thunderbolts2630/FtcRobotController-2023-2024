package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.pixelDetector.PixelDetection;
@TeleOp(name = "openCamera")
public class openCamera extends OpMode {

    @Override
    public void init() {
        PixelDetection pixelDetection =new PixelDetection(hardwareMap, telemetry);
        pixelDetection.init();
    }

    @Override
    public void loop() {

    }
}
