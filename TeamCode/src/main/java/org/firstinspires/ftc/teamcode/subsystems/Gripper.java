package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gripper implements Subsystem {
    private HardwareMap map;
    Telemetry telemetry;
    Servo serv0;
    Servo serv1;
    public Gripper(HardwareMap map, Telemetry telemetry){
        this.map=map;
        this.telemetry=telemetry;
        serv0=map.servo.get("gripper0");
        serv1=map.servo.get("gripper1");
    }
    @Override
    public void periodic() {
    }
    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }
    public Command openGripper(){
        return new RunCommand(()->{
            serv0.setPosition(0);
            serv1.setPosition(180);
        },this);
    };
    public Command closeGripper(){
        return new RunCommand(()->{
            serv0.setPosition(180);
            serv1.setPosition(0);
        },this);
    };
//    todo: maybe add toggle option

}