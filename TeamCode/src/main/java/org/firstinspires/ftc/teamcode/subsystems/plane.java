package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class plane implements Subsystem {
    private HardwareMap map;
    Servo servoPlane;

    @Config
    public static class planeCalib{
        public static double servo =0.4;
    }
    public plane(HardwareMap map){
        this.map=map;
        servoPlane =map.servo.get("plane");
        servoPlane.getController().pwmEnable();
        register();
    }
    @Override
    public void periodic() {

//        servoPlane.setPosition(planeCalib.servo);
    }

    public Command resetPlane(){
        return  new InstantCommand(()->{
            servoPlane.setPosition(0);
        });
    }
    public Command shootPlane(){
        return new InstantCommand(()->{
            servoPlane.setPosition(0.56);
        },this);
    };


}