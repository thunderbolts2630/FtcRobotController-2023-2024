package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Gripper implements Subsystem {
    private HardwareMap map;
    Telemetry telemetry;
    Servo serv0;
    Servo serv1;

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
    }
    @Override
    public void periodic() {
        serv1.setPosition(GripperConstants.servoPos.gripServoPos);
    }
    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }

    public Command openGripper(){
        return new InstantCommand(()->serv0.getController().pwmEnable()).andThen(new RunCommand(()->{

            serv0.setPosition(0.8);
            serv1.setPosition(0.8);
        },this));
    };
    public Command closeGripper(){
        return new InstantCommand(()->serv0.getController().pwmEnable()).andThen( new RunCommand(()->{
            serv0.setPosition(0.35);
            serv1.setPosition(0.35);
        },this));
    };
    public Command stop(){
        return  new InstantCommand(()->serv0.getController().pwmDisable());
    }
//    todo: maybe add toggle option

}