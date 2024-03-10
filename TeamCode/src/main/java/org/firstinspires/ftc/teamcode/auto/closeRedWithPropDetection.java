package org.firstinspires.ftc.teamcode.auto;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.PixelDetection;

@Autonomous(name = "closeRedProp")
public class closeRedWithPropDetection extends CommandOpMode {
    RobotContainer robotContainer;

    @Override
    public void initialize() {
        robotContainer = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
        robotContainer.m_pixelDetection.alliance= PixelDetection.AllianceColor.red;
        robotContainer.m_pixelDetection.init();
        waitForStart();
        Command toRun=robotContainer.centerCloseBluePath();
        switch (robotContainer.m_pixelDetection.propPos){
            case left:
                toRun=robotContainer.leftCloseRedPath();
                break;
            case right:
                toRun=robotContainer.rightCloseRedPath();
                break;
            case middle:
                toRun=robotContainer.centerCloseRedPath();
                break;
        }
        robotContainer.m_pixelDetection.closeCamera();
        toRun.schedule();
        enable();


    }

}
