package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.ArmOffset.*;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.ArmPID.*;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.ArmWights.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;
import org.firstinspires.ftc.teamcode.utils.Math.ProfileVelAcc;
import org.firstinspires.ftc.teamcode.utils.Math.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.utils.RunCommand;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Arm implements Subsystem {
    private HardwareMap map;
    private AnalogInput potentiometer1;
    private AnalogInput potentiometer2;
    private MotorEx arm1;
    private MotorEx arm2;
    private Servo servo;
    private BTController m_controller;
    VoltageSensor voltageSensor;
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
    double desired_arm1_motor_value, desired_arm2_motor_value;
    private Positions state = Positions.MIDDLEPLUS;
    private double driverAdjust1 = 0, driverAdjust2 = 0;
    private SlewRateLimiter rateLimiter;
    private boolean sensor1LimitReached = false;
    private boolean sensor2LimitReached = false;
    private double servo_desired_position = Positions.IDLE.servo;
    private double armAccBasedOffset1 = 0;
    private double armAccBasedOffset2 = 0;
    private boolean goalIsSet1 = false;
    private boolean goalIsSet2 = false;
    public boolean disableFeedFoward =true;
    ProfileVelAcc profileArm1;
    ProfileVelAcc profileArm2;
    ElapsedTime time;

    private double servoAngleFF = 0;
    private double arm2MassFromRadius = l2ff;


    public Arm(HardwareMap map,  MotorEx arm1, MotorEx arm2,VoltageSensor voltageSensor) {
        this.map = map;
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.arm1.setInverted(false);
        this.arm2.setInverted(true);
        this.voltageSensor=voltageSensor;
        m_pid2 = new ProfiledPIDController(a2KP, a2KI, a2KD, new TrapezoidProfile.Constraints(ArmProfile.maxVelocity2, ArmProfile.maxAcceleration2));
        m_pid1 = new ProfiledPIDController(a1KP, a1KI, a1KD, new TrapezoidProfile.Constraints(ArmProfile.maxVelocity1, ArmProfile.maxAcceleration1));
        m_pid1.setTolerance(8);
        m_pid2.setTolerance(10);
        rateLimiter = new SlewRateLimiter(0.3, -0.3, 0);
//        m_pid2.setTolerance(0.1);
//        m_pid1.setTolerance(0.1);
        potentiometer1 = map.get(AnalogInput.class, "pt1");//port 3
        potentiometer2 = map.get(AnalogInput.class, "pt2");//port 1
        servo = map.servo.get("armServo");

        servo.getController().pwmEnable(); //todo:uncomment
        servo.setDirection(Servo.Direction.REVERSE);
        dashboard.addData("desiredPT1", 0);
        dashboard.addData("desiredPT2", 0);
        arm1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        register();
        calculateMotorOutput();
        time=new ElapsedTime();
        profileArm1 = new ProfileVelAcc(current_first_joint_angle,time.milliseconds());
        profileArm2 = new ProfileVelAcc(current_first_joint_angle,time.milliseconds());
        servo.setPosition(0.3);
    }

    public void setMotors(double firstSpeed, double secondSpeed, double servoPos) {
        double vMax1 = 1.7 + ArmOffset.volt1Offset;
        double vMin1 = 0.6 + ArmOffset.volt1Offset;
        double voltageComp=12/voltageSensor.getVoltage();
        firstSpeed=voltageComp*firstSpeed;
        secondSpeed=voltageComp*secondSpeed;
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
        double vMin2 = 0.96 + volt2Offset;
        double vMax2 = 1.8 + volt2Offset;
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

    public double servoToDegrees(double voltage){
        servoAngleFF = ((voltage - servoVoltage2) * (servoAngle1 - servoAngle2) / (servoVoltage1 - servoVoltage2)) + servoAngle2;
        return servoAngleFF;
    }
    public void updateArm2CoM(){
        arm2MassFromRadius = Util.cosInDegrees(servoAngleFF)*gripper_weight + l2ff*second_arm_weight;
    }


    public Command armMoveDriver(DoubleSupplier speedFirst, DoubleSupplier speedSecond) {
        return new RunCommand(() -> {
            driverAdjust1 = rateLimiter.calculate(Math.pow(speedFirst.getAsDouble(), 3));
            driverAdjust2 = Math.pow(speedSecond.getAsDouble(), 3);
        });
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


    private void displayTelemtry(){
        dashboard.addData("pot1:", current_pot1_voltage);
        dashboard.addData("pot2:", current_pot2_voltage);
        dashboard.addData("angle first ", current_first_joint_angle);
        dashboard.addData("angle second", current_second_joint_angle);
        dashboard.addData("setpoint1", m_pid1.getSetpoint().position);
        dashboard.addData("setpoint2", m_pid2.getSetpoint().position);
        dashboard.addData("goal1", m_pid1.getGoal().position);
        dashboard.addData("goal2", m_pid2.getGoal().position);
        dashboard.addData("state", state.ordinal());
        dashboard.addData("arm1 FF power",arm1FF);
        dashboard.addData("arm2 FF power",arm2FF);
        dashboard.addData("arm1 velocity",profileArm1.stats.vel);
        dashboard.addData("arm1 acc",profileArm1.stats.acc);

        dashboard.addData("arm2 velocity",profileArm2.stats.vel);
        dashboard.addData("arm2 acc",profileArm2.stats.acc);

    }
    public void calculateMotorOutput(){
        servoAngleFF=servoToDegrees(servo.getPosition());
        updateArm2CoM();
        current_first_joint_angle = voltageToAngle1(potentiometer1.getVoltage());
        current_second_joint_angle = voltageToAngle2(potentiometer2.getVoltage());
        current_pot1_voltage = potentiometer1.getVoltage();
        current_pot2_voltage = potentiometer2.getVoltage();
        desired_arm1_motor_value = setMotorFromAngle1() + driverAdjust1;
        desired_arm2_motor_value = setMotorFromAngle2() + driverAdjust2;
    }
    public void updateMotorsOutput(){
        m_pid1.setIntegratorRange(MinIntegreal1, MaxIntegreal1);
        m_pid2.setIntegratorRange(MinIntegreal2, MaxIntegreal2);
        m_pid1.setIzone(aIzone1);
        m_pid2.setIzone(aIzone2);

        if (!disableFeedFoward) {
            setMotors(desired_arm1_motor_value, desired_arm2_motor_value, servo_desired_position /*servo_desired_position*/);
        }

        dashboard.update();
        m_pid1.setPID(a1KP, a1KI, a1KD);
        m_pid2.setPID(a2KP, a2KI, a2KD);
        voltFirstAngle1 = 2.371 + ArmOffset.volt1Offset;//max
        voltSecondAngle1 = 1.2 + ArmOffset.volt1Offset;//min
        voltSecondAngle2 = 1.58 + volt2Offset;
        voltFirstAngle2 = 1.13 + volt2Offset;//max
        if (!goalIsSet1) {
            m_pid1.setGoal(current_first_joint_angle);
        }
        if (!goalIsSet2) {
            m_pid2.setGoal(current_second_joint_angle);
        }

    }
    @Override
    public void periodic() {
        profileArm1.calculate(current_first_joint_angle, time.milliseconds());//for mesaurement, deleter after profile measurements
        profileArm2.calculate(current_second_joint_angle, time.milliseconds());
        servo.setPosition(calib.armServo);

        calculateMotorOutput();
//        updateMotorsOutput();
        displayTelemtry();

    }

    public double angleToVoltageA1(double angle) {
        double ptVoltage = (angle - arm1SecondAngle) * (voltFirstAngle1 - voltSecondAngle1) / (arm1FirstAngle - arm1SecondAngle) + voltSecondAngle1;
        return ptVoltage;
    }

    public double voltageToAngle1(double voltage) {
        double angle1 = ((voltage - voltSecondAngle1) * (arm1FirstAngle - arm1SecondAngle) / (voltFirstAngle1 - voltSecondAngle1)) + arm1SecondAngle;
        return angle1 + angelOffset1;
    }

    public double angleToVoltageA2(double angle) {
        double ptVoltage = ((angle - arm2SecondAngle) * (voltFirstAngle1 - voltSecondAngle1) / (arm1FirstAngle - arm1SecondAngle)) + voltSecondAngle1;
        return ptVoltage;

    }

    public double voltageToAngle2(double voltage) {
        double angle2 = ((voltage - voltSecondAngle2) * (arm2FirstAngle - arm2SecondAngle) / (voltFirstAngle2 - voltSecondAngle2)) + arm2SecondAngle;
        return angle2 + angelOffset2;
    }


    public double setMotorFromAngle1() {
        arm1PIDresult = m_pid1.calculate(current_first_joint_angle, desired_first_joint_angle);

        armAccBasedOffset1 = ((resistance *
                (RobotContainer.armAccAdjustment * Util.sinInDegrees(current_first_joint_angle))
                / (first_gear_ratio * neo_Kt)) / motorMaxVolt) / ffConv;
        arm1FF = calculateFeedForwardFirstJoint(current_first_joint_angle,current_second_joint_angle);
//        dashboard.addData("desired,current discrepancy", current_first_joint_angle-desired_first_joint_angle);
        return arm1FF + arm1PIDresult;


    }

    public double setMotorFromAngle2() {
        arm2PIDresult = m_pid2.calculate(current_second_joint_angle, desired_second_joint_angle);
        armAccBasedOffset2 = ((resistance *
                (RobotContainer.armAccAdjustment * Util.sinInDegrees(current_second_joint_angle))
                / (second_gear_ratio * neo_Kt)) / motorMaxVolt) / ffConv;
        arm2FF = calculateFeedForwardSecondJoint(current_second_joint_angle);
//        dashboard.addData("current to desired angle discrepancy 2", current_second_joint_angle-desired_second_joint_angle);
        return arm2FF + arm2PIDresult;

    }

    private double calculateFeedForwardSecondJoint(double second_joint_angle) {
        return ((resistance *
                (second_arm_weight * (g * arm2MassFromRadius * Util.cosInDegrees(second_joint_angle))
                )
                / (second_gear_ratio * neo_Kt)) / motorMaxVolt) / ffConv;
        //in volts
        // need to convert to pwm
    }

    private double calculateFeedForwardFirstJoint(double first_joint_angle, double second_joint_angle) {

        return ((resistance *g* (first_arm_weight * l1ff * Util.cosInDegrees(first_joint_angle) + arm2MassFromRadius * Util.cosInDegrees(first_joint_angle-second_joint_angle)) / (first_gear_ratio * neo_Kt))
                / motorMaxVolt)
                / ffConv;//to conv between
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
            disableFeedFoward = !disableFeedFoward;
            if (disableFeedFoward) {
                arm1.set(0);
                arm2.set(0);
                servo.getController().pwmDisable();
            }else {
                servo.getController().pwmEnable();
            }
            m_pid1.reset(current_first_joint_angle);
            m_pid2.reset(current_second_joint_angle);
        });
    }

    public Command turnOnFF() {
        return new InstantCommand(() -> {
            disableFeedFoward = false;
            servo.getController().pwmEnable();
            m_pid1.reset(current_first_joint_angle);
            m_pid2.reset(current_second_joint_angle);
        });
    }


    public Command setMiddle() {
        return goTo(Positions.MIDDLEPLUS);

    }

    public Command setLowScore() {
        return goTo(Positions.LOWSCORE);

    }

    public Command setIdle() {
        return goTo(Positions.IDLE);
    }

    public boolean ArmAtGoal() {
        return m_pid1.atGoal() && m_pid2.atGoal();
    }
    //use this to move the arm from -90 degrees to 20 degrees at 0.9 DutyCycle and then hold with FeedFoward
    public Command tuneAngle2(){
        return new InstantCommand(()->{
            disableFeedFoward =true;
            arm2.set(0.9);
        })
        .andThen(new WaitUntilCommand(()->current_second_joint_angle>10))
        .andThen(new InstantCommand(()-> {
            arm2.set(0);
            disableFeedFoward =false;
        }));
    }
    private void setState1(Positions pos) {
        m_pid1.setConstraints(pos.constraints1);
        desired_first_joint_angle = pos.angle1;
        m_pid1.reset(current_first_joint_angle);
        m_pid1.setGoal(desired_first_joint_angle);
        servo.setPosition(pos.servo);
        goalIsSet1 = true;
    }

    public void setState2(Positions pos) {
        m_pid2.setConstraints(pos.constraints2);
        desired_second_joint_angle = pos.angle2;
        m_pid2.reset(current_second_joint_angle);
        m_pid2.setGoal(desired_second_joint_angle);
        servo_desired_position = pos.servo;
        goalIsSet2 = true;
    }

    private void setStateBoth(Positions pos) {
        m_pid1.setConstraints(pos.constraints2);
        desired_first_joint_angle = pos.angle1;
        m_pid1.reset(current_first_joint_angle);
        m_pid1.setGoal(pos.angle1);
        servo.setPosition(pos.servo);
        goalIsSet1 = true;

        m_pid2.setConstraints(pos.constraints1);
        desired_second_joint_angle = pos.angle2;
        m_pid2.reset(current_second_joint_angle);
        servo_desired_position = pos.servo;
        goalIsSet2 = true;
        m_pid2.setGoal(pos.angle2);
        state = pos;

    }

    public Command goTo(Positions pos) {
        Supplier<Command> gt = () ->
                new ConditionalCommand(
                    goToState1(pos).andThen(goToState2(pos)),
                    new InstantCommand(()->setStateBoth(Positions.IDLE)).andThen(new WaitUntilCommand(()->m_pid1.atGoal() && m_pid2.atGoal())),
                    ()->pos!=Positions.IDLE
                )
                .andThen(new InstantCommand(() -> state = pos));
        return new ConditionalCommand(//return from PICKUP to front
                        new InstantCommand(()->setStateBoth(Positions.MIDPICKUP)).andThen(new WaitUntilCommand(()->m_pid1.atGoal() && m_pid2.atGoal()))
                        .andThen(goToState1(Positions.MIDDLE))
                        .andThen(goToState2(Positions.MIDDLE))
                        .andThen(new InstantCommand(() -> state = Positions.MIDDLE))
                        .andThen(gt.get()),
                gt.get(),
                () -> state == Positions.PICKUP || state == Positions.MIDPICKUP
        );

    }

    public Command setPickup() {
        return new ConditionalCommand(
                goToState1(Positions.Mid).alongWith(goToState2(Positions.Mid))
                .andThen(goToState2(Positions.MIDPICKUP))
                .andThen(goToState1(Positions.MIDPICKUP))
                .andThen(goToState2(Positions.PICKUP))
                .andThen(goToState1(Positions.PICKUP))
                .andThen(new InstantCommand(() -> state = Positions.PICKUP)),
                goTo(Positions.MIDPICKUP).andThen(goTo(Positions.PICKUP)),
                () -> !(state == Positions.MIDPICKUP || state == Positions.PICKUP));
    }


    public Command setScore() {
        return goTo(Positions.SCORE);
    }

    public Command goToState1(Positions pos) {
        return new InstantCommand(() -> setState1(pos)).andThen(new WaitUntilCommand(() -> m_pid1.atGoal()));
    }

    public Command goToState2(Positions pos) {
        return new InstantCommand(() -> setState2(pos)).andThen(new WaitUntilCommand(() -> m_pid2.atGoal()));
    }

    public Command setHighScore() {
        return goTo(Positions.HIGHSCORE);
    }

}





