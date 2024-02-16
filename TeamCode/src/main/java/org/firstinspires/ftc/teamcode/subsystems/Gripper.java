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
    private boolean isOpen2;
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
        isOpen2 = false;
    }
    @Override
    public void periodic() {
    }
    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }

    public Command openGripper() {
        return openGripper1().alongWith(openGripper2());

    }

    public Command closeGripper() {
        return closeGripper1().alongWith(closeGripper2());
    }

    public Command toggleGripper(){
        return toggleGripper1().alongWith(toggleGripper2());
    }
    public Command toggleGripper1(){
        if (isOpen1) return closeGripper1();
        else return openGripper1();
    }

    public Command toggleGripper2(){
        if (isOpen2) return closeGripper2();
        else return openGripper2();
    }

    public Command openGripper1() {
        return new InstantCommand(() -> {
            serv0.setPosition(0);
            isOpen1 = true;
            dashboard.addData("isOpen1", isOpen1);
            register();
        });
    }

    public Command closeGripper1() {
        return new InstantCommand(() -> {
            serv0.setPosition(1);
            isOpen1 = false;
            dashboard.addData("isOpen1", isOpen1);
            register();
        });
    }

    public Command closeGripper2() {
        return new InstantCommand(() -> {
            serv1.setPosition(0);
            isOpen2 = false;
            dashboard.addData("isOpen1", isOpen2);
            register();
        });
    }

    public Command openGripper2() {
        return new InstantCommand(() -> {
            serv1.setPosition(1);
            isOpen2 = true;
            dashboard.addData("isOpen1", isOpen2);
            register();
        });
    }

}