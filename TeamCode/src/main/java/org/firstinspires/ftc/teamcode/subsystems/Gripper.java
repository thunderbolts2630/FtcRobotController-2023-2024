package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
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
        return new InstantCommand(()->serv0.getController().pwmEnable()).andThen(new RunCommand(()->{

            serv0.setPosition(0.8);
//            serv1.setPosition(0.8);
        },this));
    };
    public Command closeGripper(){
        return new InstantCommand(()->serv0.getController().pwmEnable()).andThen( new RunCommand(()->{
            serv0.setPosition(0.35);
//            serv1.setPosition(0);
        },this));
    };
    public Command stop(){
        return  new InstantCommand(()->serv0.getController().pwmDisable());
    }
//    todo: maybe add toggle option

}