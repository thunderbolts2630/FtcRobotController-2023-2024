package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.ArmPID.*;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BTController;
import org.firstinspires.ftc.teamcode.utils.RunCommand;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

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
    private Positions state = Positions.MIDDLE;


    public Arm(HardwareMap map, Telemetry telemetry, MotorEx arm1, MotorEx arm2) {
        this.map = map;
        this.m_telemetry = telemetry;
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.arm1.setInverted(false);
        this.arm2.setInverted(true);
        m_pid1 = new PIDController(a1KP, a1KI, a1KD);
        m_pid2 = new PIDController(a2KP, a2KI, a2KD);
        m_pid1.setIntegratorRange(-0.1,0.1);
        m_pid2.setIntegratorRange(-0.1,0.1);
        m_pid2.setTolerance(0.2);
        potentiometer1 = map.get(AnalogInput.class, "pt1");//port 3
        potentiometer2 = map.get(AnalogInput.class, "pt2");//port 1
        servo = map.servo.get("armServo");
        servo.getController().pwmEnable();
        servo.setDirection(Servo.Direction.REVERSE);
        dashboard.addData("desiredPT1", 0);
        dashboard.addData("desiredPT2", 0);

        register();
    }

    public void setMotors(double firstSpeed, double secondSpeed, double servoPos) {
        double vMax1 = 1.5;
        double vMin1 = 0.306;
        if ((/*firstSpeed<0&& */potentiometer1.getVoltage() > vMin1) || (potentiometer1.getVoltage() > vMax1) /*&&firstSpeed>0*/) {
            arm1.set(0);

        } else {
            arm1.set(firstSpeed);

        }
        double vMin2 = 1.841;
        double vMax2 = 2.357;
        if ((potentiometer2.getVoltage() > vMax2 && secondSpeed>0)||(secondSpeed<0 && potentiometer2.getVoltage() < vMin2)) {
            arm2.set(0);
        } else {
            arm2.set(secondSpeed);

        }
        servo.setPosition(servoPos);
//        arm2.set(secondSpeed);
//        arm1.set(firstSpeed);

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
    public Command MoveArmToState(Positions pos){
        return new RunCommand(()-> {
            if (pos == Positions.idle) {
                arm1.set(0);
                arm2.set(0);
                servo.getController().pwmDisable();

            } else {
                servo.getController().pwmEnable();
                double speed1= m_pid1.calculate(potentiometer1.getVoltage(), pos.v1) + pos.ff1;
                double speed2= m_pid2.calculate(potentiometer2.getVoltage(), pos.v2) + pos.ff2;
                double speedServo=pos.servo;
                setMotors(speed1,speed2,speedServo);
                dashboard.addData("desiredPT1", Positions.DROP.v1);
                dashboard.addData("desiredPT2", Positions.DROP.v2);
            }
        });
    }
    public double ApllyFeedForward(double currentPose, double targetPose, double feedforward, double tolerance){

    }
    public Command stopManual(){
        return  new InstantCommand(()->{
            manual=true;
            arm2.set(0);
            arm1.set(0);
            servo.getController().pwmDisable();

        });
    }
    @Override
    public void periodic() {
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



