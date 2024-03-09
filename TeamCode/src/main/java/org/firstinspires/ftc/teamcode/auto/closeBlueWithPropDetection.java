package org.firstinspires.ftc.teamcode.auto;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.PixelDetection;

@Autonomous(name = "closeBlueProp")
public class closeBlueWithPropDetection extends CommandOpMode {
    RobotContainer robotContainer;

    @Override
    public void initialize() {
        robotContainer = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
        robotContainer.m_pixelDetection.alliance= PixelDetection.AllianceColor.blue;
        robotContainer.m_pixelDetection.init();
        waitForStart();
        Command toRun=robotContainer.blueCloseCenterPath();
        switch (robotContainer.m_pixelDetection.propPos){
            case left:
                toRun=robotContainer.blueCloseLeftPath();
                break;
            case right:
                toRun=robotContainer.blueCloseRightPath();
                break;
            case middle:
                toRun=robotContainer.blueCloseCenterPath();
                break;

        }
        robotContainer.m_pixelDetection.closeCamera();
        toRun.schedule();
        enable();


    }

}
