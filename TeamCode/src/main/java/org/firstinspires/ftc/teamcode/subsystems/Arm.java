package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.ArmPID.*;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.ArmWights.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BTController;
import org.firstinspires.ftc.teamcode.utils.Math.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.utils.RunCommand;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.PID.*;
import org.firstinspires.ftc.teamcode.utils.Util;
import org.firstinspires.ftc.teamcode.utils.cheesy.InterpolatingDouble;

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
    private ProfiledPIDController m_pid1;
    private ProfiledPIDController m_pid2;
    private double desired_first_joint_angle = 90;
    private double desired_second_joint_angle = -90;
    private Translation2d current_desired_point;
    private double current_second_joint_angle;
    private double current_first_joint_angle;
    private double current_pot1_voltage;
    private double current_pot2_voltage;
    private double arm1PIDresult, arm1FF;
    private double arm2PIDresult, arm2FF;
    private boolean manual = true;
    double desired_arm1_motor_value, desired_arm2_motor_value;
    private Positions state = Positions.idle;
    private double driverAdjust1 = 0, driverAdjust2 = 0;
    private SlewRateLimiter rateLimiter;

    private boolean sensor1LimitReached = false;
    private boolean sensor2LimitReached = false;

    public Arm(HardwareMap map, Telemetry telemetry, MotorEx arm1, MotorEx arm2) {
        this.map = map;
        this.m_telemetry = telemetry;
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.arm1.setInverted(false);
        this.arm2.setInverted(true);
        m_pid2 = new ProfiledPIDController(a2KP, a2KI, a2KD, new TrapezoidProfile.Constraints(ArmProfile.maxVelocity2, ArmProfile.maxAcceleration2));
        m_pid1 = new ProfiledPIDController(a1KP, a1KI, a1KD, new TrapezoidProfile.Constraints(ArmProfile.maxVelocity1, ArmProfile.maxAcceleration1));

        rateLimiter = new SlewRateLimiter(0.3, -0.3, 0);
//        m_pid2.setTolerance(0.1);
//        m_pid1.setTolerance(0.1);
        potentiometer1 = map.get(AnalogInput.class, "pt1");//port 3
        potentiometer2 = map.get(AnalogInput.class, "pt2");//port 1
        servo = map.servo.get("armServo");
        servo.getController().pwmEnable();
        servo.setDirection(Servo.Direction.REVERSE);
        dashboard.addData("desiredPT1", 0);
        dashboard.addData("desiredPT2", 0);
        arm1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        register();
    }

    public void     setMotors(double firstSpeed, double secondSpeed, double servoPos) {
        double vMax1 = 1.7 + ArmOffset.volt1Offset;
        double vMin1 = 0.6 + ArmOffset.volt1Offset;
        if (potentiometer1.getVoltage() < 0.4 || potentiometer1.getVoltage() > 2.9 || sensor1LimitReached) {
            arm1.set(0);
            sensor1LimitReached = true;
        } else if ((firstSpeed < 0 && potentiometer1.getVoltage() < vMin1) || potentiometer1.getVoltage() > vMax1) {
            arm1.set(firstSpeed - (driverAdjust1 * 0.9));
            dashboard.addData("stpped", 0);
        } else {
            arm1.set(firstSpeed);

            dashboard.addData("stpped", firstSpeed);

        }
        double vMin2 = 0.96;
        double vMax2 = 1.8;
        if (potentiometer2.getVoltage() < 0.5 || potentiometer2.getVoltage() > 2.9 || sensor2LimitReached) {
            arm2.set(0);
            sensor2LimitReached = true;
        } else if ((potentiometer2.getVoltage() > vMax2 && secondSpeed > 0)) {
            arm2.set(driverAdjust2);
        } else {

            arm2.set(secondSpeed);

        }
        servo.setPosition(servoPos);
//        arm2.set(secondSpeed);
//        arm1.set(firstSpeed);

    }


    public Command armMoveDriver(DoubleSupplier speedFirst, DoubleSupplier speedSecond) {
        return new RunCommand(() -> {
            driverAdjust1 = rateLimiter.calculate(Math.pow(speedFirst.getAsDouble(), 3));
            driverAdjust2 = Math.pow(speedSecond.getAsDouble(), 3);
        });
    }


    public Command setState(Positions state) {
        return new InstantCommand(() -> this.state = state);
    }

    public double ApplyFeedForward(double currentPose, double targetPose, double feedforward, double tolerance) {
        if (Math.abs(currentPose - targetPose) < tolerance) {
            return 0;
        } else {
            return feedforward;
        }
    }

    public Command stopAdjust() {
        return new InstantCommand(() -> {
            driverAdjust2 = 0;
            driverAdjust1 = 0;
        });
    }


    @Override
    public void periodic() {
        m_pid1.setIntegratorRange(MinIntegreal1, MaxIntegreal1);
        m_pid2.setIntegratorRange(MinIntegreal2, MaxIntegreal2);
        m_pid1.setIzone(aIzone1);
        m_pid2.setIzone(aIzone2);
        current_first_joint_angle = voltageToAngle1(potentiometer1.getVoltage());
        current_second_joint_angle = voltageToAngle2(potentiometer2.getVoltage());
        current_pot1_voltage = potentiometer1.getVoltage();
        current_pot2_voltage = potentiometer2.getVoltage();
        desired_arm1_motor_value = setMotorFromAngle1() + driverAdjust1;
        desired_arm2_motor_value = setMotorFromAngle2() + driverAdjust2;
        if (!manual) {
            setMotors(desired_arm1_motor_value, desired_arm2_motor_value, calib.armServo);
        }
        dashboard.addData("desired angle 1:", desired_first_joint_angle);
        dashboard.addData("desired angle 2:", desired_second_joint_angle);
        dashboard.addData("pot1:", current_pot1_voltage);
        dashboard.addData("pot2:", current_pot2_voltage);
        dashboard.addData("first angle ", current_first_joint_angle );
        dashboard.addData("second angle", current_second_joint_angle );
        dashboard.addData("arm1PID", arm1PIDresult);
        dashboard.addData("arm2PID", arm2PIDresult);
        dashboard.addData("power to arm 1", desired_arm1_motor_value);
        dashboard.addData("power to arm 2", desired_arm2_motor_value);
        dashboard.addData("arm1 ticks", arm1.getCurrentPosition());
        dashboard.addData("arm total error1",m_pid1.getTotalError());
        dashboard.addData("arm total error2",m_pid2.getTotalError());

        dashboard.update();
        m_pid1.setPID(a1KP, a1KI, a1KD);
        m_pid2.setPID(a2KP, a2KI, a2KD);
        voltFirstAngle1 = 2.371 + ArmOffset.volt1Offset;//max
        voltSecondAngle1 = 1.2 + ArmOffset.volt1Offset;//min

    }

    public double angleToVoltageA1(double angle) {
        double ptVoltage = (angle - arm1SecondAngle) * (voltFirstAngle1 - voltSecondAngle1) / (arm1FirstAngle - arm1SecondAngle) + voltSecondAngle1;
        return ptVoltage;
    }

    public double voltageToAngle1(double voltage) {
        double angle1 = ((voltage - voltSecondAngle1) * (arm1FirstAngle - arm1SecondAngle) / (voltFirstAngle1 - voltSecondAngle1)) + arm1SecondAngle;
        return angle1+angelOffset1;
    }

    public double angleToVoltageA2(double angle) {
        double ptVoltage = ((angle - arm2SecondAngle) * (voltFirstAngle1 - voltSecondAngle1) / (arm1FirstAngle - arm1SecondAngle)) + voltSecondAngle1;
        return ptVoltage;

    }

    public double voltageToAngle2(double voltage) {
        double angle2 = ((voltage - voltSecondAngle2) * (arm2FirstAngle - arm2SecondAngle) / (voltFirstAngle2 - voltSecondAngle2)) + arm2SecondAngle;
        return angle2+angelOffset2;
    }


    public double setMotorFromAngle1() {
        arm1PIDresult = m_pid1.calculate(current_first_joint_angle, desired_first_joint_angle);

        arm1FF = calculateFeedForwardFirstJoint(current_first_joint_angle);
        dashboard.addData("desired,current discrepancy", current_first_joint_angle-desired_first_joint_angle);
        return arm1FF + arm1PIDresult;

    }

    public double setMotorFromAngle2() {
        arm2PIDresult = m_pid2.calculate(current_second_joint_angle, desired_second_joint_angle);
        arm2FF = calculateFeedForwardSecondJoint(current_second_joint_angle);
        dashboard.addData("current to desired angle discrepancy 2", current_second_joint_angle-desired_second_joint_angle);
        return arm2FF + arm2PIDresult;

    }

    private double calculateFeedForwardSecondJoint(double second_joint_angle) {
        return ((resistance *
                (second_arm_weight * (g * l2 * Util.cosInDegrees(second_joint_angle))
                )
                / (second_gear_ratio * neo_Kt)) / motorMaxVolt) / ffConv;
        //in volts
        // need to convert to pwm
    }

    private double calculateFeedForwardFirstJoint(double first_joint_angle) {

        return ((resistance *
                (first_arm_weight * (g * l1 * Util.cosInDegrees(first_joint_angle ))
                )
                / (first_gear_ratio * neo_Kt)) / motorMaxVolt) / ffConv;//to conv between
        // in volts
    }


    //    voltage_90_degrees = resistance_motor*torque_90_degrees/(gear_ratio*Kt)
    //calculates the feedForward to second joint using a torque calculation to the current angle
    private void setDesiredAnglesToJointsPositiveX() {
        desired_second_joint_angle = -Util.aCosInDegrees(
                (Math.pow(current_desired_point.getX(), 2)
                        + Math.pow(current_desired_point.getY(), 2)
                        - Math.pow(l1, 2) - Math.pow(l2, 2))
                        / (2 * l1 * l2));
        desired_first_joint_angle =
                Math.toDegrees(Math.atan2(
                                current_desired_point.getY(),
                                current_desired_point.getX()
                        )

                )
                        - Math.toDegrees(
                        Math.atan2(
                                l2 * Util.sinInDegrees(desired_second_joint_angle)
                                , (l1 + l2 * Util.cosInDegrees(desired_second_joint_angle))
                        )
                );
        desired_second_joint_angle = desired_first_joint_angle + desired_second_joint_angle;
    }

    private void setDesiredAnglesToJointsNegativeX() {
        desired_second_joint_angle = -Util.aCosInDegrees(
                (Math.pow(current_desired_point.getX(), 2)
                        + Math.pow(current_desired_point.getY(), 2)
                        - Math.pow(l1, 2) - Math.pow(l2, 2))
                        / (2 * l1 * l2));
        desired_first_joint_angle =
                Math.toDegrees(Math.atan2(
                                current_desired_point.getY(),
                                current_desired_point.getX()
                        )

                )
                        + Math.toDegrees(
                        Math.atan2(
                                l2 * Util.sinInDegrees(desired_second_joint_angle)
                                , (l1 + l2 * Util.cosInDegrees(desired_second_joint_angle))
                        )
                );
        desired_second_joint_angle = desired_first_joint_angle - desired_second_joint_angle;
    }

    public Translation2d anglesToPoint(double firstAngle, double second) {
        double x = Util.cosInDegrees(firstAngle) * l1 + Util.cosInDegrees(180 - second + firstAngle) * l2;
        double y = Util.sinInDegrees(firstAngle) * l1 + Util.sinInDegrees(180 - second + firstAngle) * l2;
        y = ((int) (y * 1000)) / 1000.0;
        x = ((int) (x * 1000)) / 1000.0;
        return new Translation2d(x, y);
    }


    public void setDesiredPoint(Translation2d point) {
        //checks if the given point is already the desired point
        if (!point.equals(current_desired_point)) {
            // checks if the given point is in bounds of possibly
            //checks if point is not in the ground
            if (point.getY() > -0.05) {
                //checks if point is in arm radius
                if (Math.hypot(point.getX(), point.getY()) < l1 + l2) {
                    current_desired_point = point;
                    //set angles accordingly -x and x, not the same calculation for both
                    if (current_desired_point.getX() >= 0) {
                        setDesiredAnglesToJointsPositiveX();
                    } else {
                        setDesiredAnglesToJointsNegativeX();
                    }
                }
            }
        }

    }

    public Command toggleFF() {
        return new InstantCommand(() -> {
            manual = !manual;
            if (manual) {
                arm1.set(0);
                arm2.set(0);
            }
            m_pid1.reset(current_first_joint_angle);
            m_pid2.reset(current_second_joint_angle);
        });
    }

    public BTCommand armMoveToPoint(Translation2d point) {
        return new RunCommand(
                () -> {
                    setDesiredPoint(point);
                }
        );
    }

    public Command setDesiredAngle() {

        return new InstantCommand(() -> {
            m_pid2.setConstraints(new TrapezoidProfile.Constraints(ArmProfile.maxVelocity2, ArmProfile.maxAcceleration2));
             m_pid1.setConstraints(new TrapezoidProfile.Constraints(ArmProfile.maxVelocity1, ArmProfile.maxAcceleration1));
            if (a1DesAngle > 80 || a1DesAngle < 140) {
                desired_first_joint_angle = a1DesAngle;
                m_pid1.reset(current_first_joint_angle);
                m_pid1.setGoal(a1DesAngle);
            }
            if (a2DesAngle > -100 || a2DesAngle < 30) {
                desired_second_joint_angle = a2DesAngle;
                m_pid2.reset(current_second_joint_angle);
                m_pid2.setGoal(a2DesAngle);
            }
        });
    }
}



