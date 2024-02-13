package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.ArmPID.*;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BTController;
import org.firstinspires.ftc.teamcode.utils.RunCommand;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.PID.*;

import java.util.function.DoubleSupplier;

public class Arm implements Subsystem {
    private HardwareMap map;
    private AnalogInput potentiometer1;
    private AnalogInput potentiometer2;
    private Telemetry m_telemetry;
    private MotorEx arm1;
    private MotorEx arm2;
    private Servo servo;
    private BTController m_controller;
    private Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
    private PIDController m_pid1;
    private PIDController m_pid2;
    private double desired_first_joint_angle = 0;
    private double desired_second_joint_angle = 0;
    private Translation2d current_desired_point;
    private double current_second_joint_angle;
    private double current_first_joint_angle;
    private double current_pot1_voltage;
    private double current_pot2_voltage;
    private double arm1PIDresult, arm1FF;
    private double arm2PIDresult, arm2FF;
    private boolean manual=true;
    private Positions state = Positions.IDLE;


    public Arm(HardwareMap map, Telemetry telemetry, MotorEx arm1, MotorEx arm2) {
        this.map = map;
        this.m_telemetry = telemetry;
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.arm1.setInverted(false);
        this.arm2.setInverted(true);
        m_pid1 = new PIDController(a1KP, a1KI, a1KD);
        m_pid2 = new PIDController(a2KP, a2KI, a2KD);
        potentiometer1 = map.get(AnalogInput.class, "pt1");//port 3
        potentiometer2 = map.get(AnalogInput.class, "pt2");//port 1
        servo = map.servo.get("armServo");
        servo.getController().pwmEnable();
        register();

    }



    public double angleToVoltageA1(double angle) {
        double ptVoltage = (angle - arm1Min) * (vMax1 - vMin1) / (a1Max - arm1Min) + vMin1;
        return ptVoltage;
    }

    public double voltageToAngle1(double voltage) {
        double angle1 = (voltage - vMin1) * (a1Max - arm1Min) / (vMax1 - vMin1) + arm1Min;
        return angle1;
    }

    public double angleToVoltageA2(double angle) {
        double ptVoltage = (angle - arm2Min) * (vMax1 - vMin1) / (a1Max - arm1Min) + vMin1;
        return ptVoltage;

    }

    public double voltageToAngle2(double voltage) {
        double angle2 = (voltage - vMin2) * (a2Max - arm2Min) / (vMax2 - vMin2) + arm2Min;
        return angle2;
    }

    public void setMotors(double firstSpeed, double secondSpeed, double servoPos) {
//        if (potentiometer1.getVoltage() > vMax1 || potentiometer1.getVoltage() < vMin1) {
//            arm1.set(0);
//
//        } else {
//            arm1.set(firstSpeed);
//
//        }
//        if (potentiometer2.getVoltage() > vMax2 || potentiometer2.getVoltage() < vMin2) {
//            arm2.set(0);
//        } else {
//            arm2.set(secondSpeed);
//
//        }
        servo.setPosition(0.6);
        arm2.set(secondSpeed);
        arm1.set(firstSpeed);

    }


    public Command armMoveManual(DoubleSupplier speedFirst, DoubleSupplier speedSecond, DoubleSupplier posServo) {
        return new RunCommand(() -> {
            manual = true;
//            arm2PID = m_pid2.calculate(current_second_joint_angle, desired_second_joint_angle);
//            arm2FF = calculateFeedForwardSecondJoint(current_second_joint_angle);
//            arm1PID = m_pid1.calculate(current_first_joint_angle, desired_second_joint_angle);
//            arm1FF = calculateFeedForwardFirstJoint(current_first_joint_angle);
            arm1.set(calib.arm1);
            arm2.set(calib.arm2);
            servo.setPosition(calib.armServo);
        });
    }
    public BTCommand stopManual(){
        return  new RunCommand(()->{
            manual=true;
            arm2.set(0);
            arm1.set(0);

        });
    }
    @Override
    public void periodic() {
        current_first_joint_angle = voltageToAngle1(potentiometer1.getVoltage());
        current_second_joint_angle = voltageToAngle2(potentiometer2.getVoltage());
        current_pot1_voltage = potentiometer1.getVoltage();
        current_pot2_voltage = potentiometer2.getVoltage();
//        if (!manual) {
//            setMotorFromAngle2();
//            setMotorFromAngle1();
//        }
        dashboard.addData("desired angle 1:", desired_first_joint_angle);
        dashboard.addData("desired angle 2:", desired_second_joint_angle);
        dashboard.addData("pot1:", current_pot1_voltage);
        dashboard.addData("pot2:", current_pot2_voltage);
        dashboard.addData("first angle ", current_first_joint_angle);
        dashboard.addData("second angle", current_second_joint_angle);
        dashboard.addData("arm1PID", arm1PIDresult);
        dashboard.addData("arm2PID", arm2PIDresult);
        dashboard.update();

        m_pid1.setPID(a1KP, a1KI, a1KD);
        m_pid2.setPID(a2KP, a2KI, a2KD);
    }



}



