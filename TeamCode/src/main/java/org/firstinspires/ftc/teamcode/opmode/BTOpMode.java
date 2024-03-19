package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonLynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.BT.hardware.BTLynxDCMotorController;

import java.util.List;
import java.util.Objects;

//used for the Command Scheduler and replacing motors in the hardware map with our own

public class BTOpMode extends LinearOpMode {
    RobotContainer robotContainer;
    ElapsedTime loopTime;
    Telemetry dashboard=FtcDashboard.getInstance().getTelemetry();
    @Override
    public void runOpMode() throws InterruptedException {
        robotContainer= new RobotContainer(hardwareMap,telemetry,gamepad1,gamepad2);
        initialize();
        Robot.enable();
        CommandScheduler.getInstance().enable();
        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            loopTime.reset();
            CommandScheduler.getInstance().run();
            robotContainer.clearSensorsCache();
            dashboard.addData("loop time ms",loopTime.milliseconds());
            dashboard.update();
        }
        CommandScheduler.getInstance().reset();
    }

    public void initialize() {
        //override this for autonomous OpModes
        //use this to set the color of the robot (red/blue)
        //and schedule which command to run based on the prop detection
    }
}
