package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotContainer;
@Disabled
//@Autonomous(name = "CenterFarRed")
public class CenterFarRed extends CommandOpMode {
    RobotContainer robotContainer;
    @Override
    public void initialize() {
        robotContainer = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
        waitForStart();
        robotContainer.CenterFarRedPath().schedule();
        enable();

    }
}
