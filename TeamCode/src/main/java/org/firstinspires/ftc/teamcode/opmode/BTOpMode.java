package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;

//used for the Command Scheduler and replacing motors in the hardware map with our own

public class BTOpMode extends LinearOpMode {
    RobotContainer robotContainer;
    ElapsedTime loopTime;
    Telemetry dashboard=FtcDashboard.getInstance().getTelemetry();
    public static double msloopPeriod=20; //the desired loop time in ms
    private double currentLoopTime=0;
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
            dashboard.update();
            currentLoopTime=loopTime.milliseconds();
            dashboard.addData("loop time ms",currentLoopTime);
            if(currentLoopTime<msloopPeriod){
                //sleep for the rest of the desired period
                sleep( (int)msloopPeriod-(int)currentLoopTime);
            }
        }
        CommandScheduler.getInstance().reset();
    }

    public void initialize() {
        //override this for autonomous OpModes
        //use this to set the color of the robot (red/blue)
        //and schedule which command to run based on the prop detection
    }
}
