package org.firstinspires.ftc.teamcode.auto;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotContainer;
@Autonomous(name="centerAutoBlue", group = "Linear Opmode")
public class centerCloseBlueAuto extends CommandOpMode {
    RobotContainer robotContainer;

    @Override
    public void initialize() {
        robotContainer = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
        waitForStart();
        robotContainer.closeBlueCenterAutoBT().schedule();
        enable();

    }
}

