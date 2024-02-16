package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.teamcode.Constants;

public class Gripper implements Subsystem {
    private HardwareMap map;
    Telemetry telemetry;
    Servo serv0;
    Servo serv1;
    private boolean isOpen1;
    private boolean isOpen0;
    private Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();



    public static class GripperConstants{
        @Config public static class servoPos{
            public static double gripServoPos=0.0;
        }
    }
    public Gripper(HardwareMap map, Telemetry telemetry){
        this.map=map;
        this.telemetry=telemetry;
        serv0=map.servo.get("gripper0");
        serv1=map.servo.get("gripper1");
        register();
        serv1.getController().pwmEnable();
        isOpen1 = false;
        isOpen0 = false;

    }
    @Override
    public void periodic() {
        serv0.setPosition(isOpen0?0.8:0.3);
        serv1.setPosition(isOpen1?0.05:0.33);
    }
    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }

    public Command openGripper() {
        return openGripper1().alongWith(openGripper0());

    }

    public Command closeGripper() {
        return closeGripper1().alongWith(closeGripper0());
    }

    public Command toggleGripper(){
        return toggleGripper1().alongWith(toggleGripper0());
    }
    public Command toggleGripper1(){
        return  new InstantCommand(()->{
           isOpen1=!isOpen1; 
        });
    }

    public Command toggleGripper0(){
        return new InstantCommand(()->{
           isOpen0=!isOpen0; 
        });
    }

    public Command openGripper1() {
        return new InstantCommand(() -> {
            isOpen1 = true;
            dashboard.addData("isOpen1", isOpen1);
        });
    }

    public Command closeGripper1() {
        return new InstantCommand(() -> {
            isOpen1 = false;
            dashboard.addData("isOpen1", isOpen1);
        });
    }

    public Command closeGripper0() {
        return new InstantCommand(() -> {
            isOpen0 = false;
            dashboard.addData("isOpen1", isOpen0);
        });
    }

    public Command openGripper0() {
        return new InstantCommand(() -> {
            isOpen0 = true;
            dashboard.addData("isOpen1", isOpen0);
        });
    }

}