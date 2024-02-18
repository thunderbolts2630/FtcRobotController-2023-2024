package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotContainer;

@Autonomous(name="FarBlueAuto", group = "Linear Opmode")
public class closeBlueAuto extends LinearOpMode {
    RobotContainer robotContainer = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
    @Override
    public void runOpMode() throws InterruptedException {
        robotContainer.CloseBlueAuto();
    }
}
