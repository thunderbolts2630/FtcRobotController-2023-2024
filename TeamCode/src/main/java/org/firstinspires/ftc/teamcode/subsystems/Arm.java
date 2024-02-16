package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.ArmPID.*;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.Constants.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BTController;
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
    double desired_arm1_motor_value,desired_arm2_motor_value;
    private Positions state = Positions.idle;


    public Arm(HardwareMap map, Telemetry telemetry, MotorEx arm1, MotorEx arm2) {
        this.map = map;
        this.m_telemetry = telemetry;
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.arm1.setInverted(false);
        this.arm2.setInverted(true);
        m_pid1 = new PIDController(a1KP, a1KI, a1KD);
        m_pid2 = new PIDController(a2KP, a2KI, a2KD);

//        m_pid2.setTolerance(0.1);
//        m_pid1.setTolerance(0.1);
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
        double vMax1 = 2.2;
        double vMin1 = 1.02;
//        if ((firstSpeed>0 && potentiometer1.getVoltage() < vMin1) || (potentiometer1.getVoltage() > vMax1) &&firstSpeed<0) {
//            arm1.set(0);
//            dashboard.addData("stpped",0);
//        } else {
//            arm1.set(firstSpeed);
//            dashboard.addData("stpped",firstSpeed);
//
//        }
//        double vMin2 = 0.65;
//        double vMax2 = 1.33;
//        if ((potentiometer2.getVoltage() > vMax2 && secondSpeed<0)||(secondSpeed>0 && potentiometer2.getVoltage() < vMin2)) {
//            arm2.set(0);
//        } else {
//            arm2.set(secondSpeed);
//
//        }
        servo.setPosition(servoPos);
        arm2.set(secondSpeed);
        arm1.set(firstSpeed);

    }


    public Command armMoveManual(DoubleSupplier speedFirst, DoubleSupplier speedSecond) {
        return new RunCommand(() -> {
            manual = true;
//            arm2PID = m_pid2.calculate(current_second_joint_angle, desired_second_joint_angle);
//            arm2FF = calculateFeedForwardSecondJoint(current_second_joint_angle);
//            arm1PID = m_pid1.calculate(current_first_joint_angle, desired_second_joint_angle);
//            arm1FF = calculateFeedForwardFirstJoint(current_first_joint_angle);
            setMotors(Math.pow(speedFirst.getAsDouble(),3)+state.ff1, Math.pow(speedSecond.getAsDouble(),3)+state.ff2, state.servo);
            dashboard.addData("speed1+ff", speedFirst.getAsDouble()+state.ff1);
            dashboard.addData("speed2+ff", speedSecond.getAsDouble()+state.ff2);

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
                double ff1= 0;//kFFMap1.getInterpolated(new InterpolatingDouble(potentiometer1.getVoltage())).value;
                double ff2= 0;//kFFMap2.getInterpolated(new InterpolatingDouble(potentiometer2.getVoltage())).value;
                double speed1= m_pid1.calculate(potentiometer1.getVoltage()-Positions.MIDDLE.v1, pos.v1-Positions.MIDDLE.v1)+ff1;
                double speed2= m_pid2.calculate(potentiometer2.getVoltage(), pos.v2)+ ff2;
                double speedServo=pos.servo;
                setMotors(speed1,speed2,speedServo);
                dashboard.addData("desiredPT1", Positions.DROP.v1);
                dashboard.addData("desiredPT2", Positions.DROP.v2);
                dashboard.addData("speed1", speed1);
                dashboard.addData("ff1", ff1);
                dashboard.addData("ff2", ff2);
                dashboard.addData("speed2", speed2);

            }
        });
    }
    public Command setState(Positions state){
        return  new InstantCommand(()->this.state=state);
    }
    public double ApplyFeedForward(double currentPose, double targetPose, double feedforward, double tolerance){
        if(Math.abs(currentPose-targetPose)<tolerance){
            return 0;
        }else{
            return feedforward;
        }
    }
    public Command stopManual(){
        return  new InstantCommand(()->{
            manual=true;
            arm2.set(0);
            arm1.set(0);
            servo.getController().pwmDisable();
            m_pid2.reset();
            m_pid1.reset();
        });
    }
    @Override
    public void periodic() {
        m_pid1.setIntegratorRange(-Constants.ArmConstants.calib.MaxIntegreal,Constants.ArmConstants.calib.MaxIntegreal);
        m_pid2.setIntegratorRange(-Constants.ArmConstants.calib.MaxIntegreal,Constants.ArmConstants.calib.MaxIntegreal);
        current_first_joint_angle = voltageToAngle1(potentiometer1.getVoltage());
        current_second_joint_angle = voltageToAngle2(potentiometer2.getVoltage());
        current_pot1_voltage = potentiometer1.getVoltage();
        current_pot2_voltage = potentiometer2.getVoltage();
        desired_arm1_motor_value=setMotorFromAngle2();
        desired_arm2_motor_value=setMotorFromAngle1();
//        setMotors(desired_arm1_motor_value,desired_arm2_motor_value,0.33);

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

    public double angleToVoltageA1(double angle) {
        double ptVoltage = (angle - arm1SecondAngle) * (voltFirstAngle1 - voltSecondAngle1) / (arm1FirstAngle - arm1SecondAngle) + voltSecondAngle1;
        return ptVoltage;
    }

    public double voltageToAngle1(double voltage) {
        double angle1 = (voltage - voltSecondAngle1) * (arm1FirstAngle - arm1SecondAngle) / (voltFirstAngle1 - voltSecondAngle1) + arm1SecondAngle;
        return angle1;
    }

    public double angleToVoltageA2(double angle) {
        double ptVoltage = (angle - arm2SecondAngle) * (voltFirstAngle1 - voltSecondAngle1) / (arm1FirstAngle - arm1SecondAngle) + voltSecondAngle1;
        return ptVoltage;

    }

    public double voltageToAngle2(double voltage) {
        double angle2 = (voltage - voltSecondAngle2) * (arm2FirstAngle - arm2SecondAngle) / (voltFirstAngle2 - voltSecondAngle2) + arm2SecondAngle;
        return angle2;
    }

    private double calculateFeedForwardFirstJoint(double first_joint_angle) {

        return ((resistance *
                (first_arm_weight * (g * l1 * Util.cosInDegrees(first_joint_angle))
                )
                / (first_gear_ratio * neo_Kt)) / motorMaxVolt) / ffConv;//to conv between
        // in volts
    }

    public double setMotorFromAngle1() {
        arm1PIDresult = m_pid1.calculate(current_first_joint_angle, desired_first_joint_angle);
        arm1FF = calculateFeedForwardFirstJoint(current_first_joint_angle);
        dashboard.addData("first joint output:", arm1PIDresult);
        return  c_arm1FF.calculate(arm1FF + arm1PIDresult);

    }

    public double setMotorFromAngle2() {
        arm2PIDresult = m_pid2.calculate(current_second_joint_angle, desired_second_joint_angle);
        arm2FF = calculateFeedForwardSecondJoint(current_second_joint_angle);
        dashboard.addData("second joint output:", arm2PIDresult);
        return c_arm2FF.calculate(arm2FF + arm2PIDresult);

    }
    private double calculateFeedForwardSecondJoint(double second_joint_angle) {
        return ((resistance *
                (second_arm_weight * (g * l2 * Util.cosInDegrees(second_joint_angle + 90))
                )
                / (second_gear_ratio * neo_Kt)) / motorMaxVolt) / ffConv;
        //in volts
        // need to convert to pwm
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

    public BTCommand armMoveToPoint(Translation2d point) {
        return new RunCommand(
                () -> {
                    setDesiredPoint(point);
                }
        );
    }


}



