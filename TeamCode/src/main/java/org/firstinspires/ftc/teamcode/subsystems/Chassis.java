package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.BT.BTHolonomicDriveController;
import org.firstinspires.ftc.teamcode.utils.BT.BTposeEstimator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
import org.firstinspires.ftc.teamcode.utils.RunCommand;
import org.firstinspires.ftc.teamcode.utils.geometry.BTPose2d;
import org.firstinspires.ftc.teamcode.utils.geometry.BTRotation2d;
import org.firstinspires.ftc.teamcode.utils.geometry.BTTransform2d;
import org.firstinspires.ftc.teamcode.utils.geometry.BTTranslation2d;

import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.RotationPID.*;
import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.*;
import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.PIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Chassis.ChassisMotorsFeedfoward.*;
import static org.firstinspires.ftc.teamcode.subsystems.Chassis.Tune.ChassisPower;
import static org.firstinspires.ftc.teamcode.subsystems.Chassis.Tune.useTune;

import java.util.function.DoubleSupplier;

public class Chassis implements Subsystem {
    public static boolean hasReset=false;

    private VoltageSensor voltage_sensor;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private PIDFController m_pidcontroller;
    private PIDController m_rotationpid;
    private SimpleMotorFeedforward m_rotFF;
    private double prevTime = 0;
    private BTTransform2d velocity, prevVelocity = new BTTransform2d(), acceleration, prevAcceleration = new BTTransform2d();
    private BTPose2d prevPos;
    private HardwareMap map;
    private BTposeEstimator odometry;
    private MotorEx motor_FL;
    private MotorEx motor_FR;
    private MotorEx motor_BL;
    private MotorEx motor_BR;
    private RevIMU gyro;
    private BTPose2d m_postitionFromTag;
    private double maxVelocityX = 0;
    private double maxVelocityY = 0;
    private double maxVelocityTheta = 0;
    private double maxAccelerationX = 0;
    private double maxAccelerationTheta = 0;
    private double maxAccelerationY = 0;
    private PIDController m_pidX;
    private Motor.Encoder m_leftEncoder;
    private Motor.Encoder m_centerEncoder;
    private Motor.Encoder m_rightEncoder;
    public double RobotXAcc = 0;
    public int test = 0;

    ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private PIDController m_pidY;

    public static class ChassisMotorsFeedfoward{
        @Config
        public static class chassisFL{
            public static double ks=0.085;//0.085 is a value from kookybotz
            public static double kv=1;
        }@Config
        public static class chassisFR{
            public static double ks=0.085;//0.085 is a value from kookybotz
            public static double kv=1;
        }@Config
        public static class chassisBL{
            public static double ks=0.085;//0.085 is a value from kookybotz
            public static double kv=1;
        }@Config
        public static class chassisBR{
            public static double ks=0.085;//0.085 is a value from kookybotz
            public static double kv=1;
        }
    }


    public Chassis(HardwareMap map, MotorEx.Encoder center, MotorEx.Encoder rightEncoder,VoltageSensor voltage_sensor) {
        this.map = map;
        motor_FL = new MotorEx(map, "motor_FL");//1
        motor_FR = new MotorEx(map, "motor_FR");//2
        motor_BL = new MotorEx(map, "motor_BL");//0
        motor_BR = new MotorEx(map, "motor_BR");//3
        this.voltage_sensor=voltage_sensor;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro = new RevIMU(map, "imu");
        gyro.init(parameters);
        if(!hasReset) {
            gyro.reset();
            hasReset=true;
        }
        gyro.invertGyro();
        m_pidX = new PIDController(Xkp,Xki,Xkd);
        m_pidY = new PIDController(Ykp,Yki,Ykd);

        motor_FL.setInverted(false);
        motor_BL.setInverted(false);
        motor_BR.setInverted(true);
        motor_FR.setInverted(true);

        motor_FR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_BR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_BL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_FL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_leftEncoder =center;
        m_centerEncoder =  new MotorEx(map, "encoderLeft").encoder;// was switched dont have time ot solve 
        m_rightEncoder = rightEncoder;
        odometry = new BTposeEstimator(
                () -> metersFormTicks(m_leftEncoder.getPosition()),//center and left switched
                () -> metersFormTicks(m_rightEncoder.getPosition()),
                () -> metersFormTicks(m_centerEncoder.getPosition()),
                () -> gyro.getHeading(),
                TRACKWIDTH, WHEEL_OFFSET);
        prevPos = odometry.getPose();
        time.reset();
        time.startTime();
        m_rotFF = new SimpleMotorFeedforward(rks,1);
        prevTime = time.time();
        m_pidcontroller = new PIDFController(kp, ki, kd, kff);
        m_rotationpid = new PIDController(rkp,rki,rkd);
        m_rotationpid.enableContinuousInput(-180,180);
        m_rotationpid.setTolerance(tolerance);
        register();
        dashboardTelemetry.addData("drive: ", 0);
        dashboardTelemetry.addData("frontVelocityAuto", 0);
        dashboardTelemetry.addData("sideVelocityAuto", 0);
        dashboardTelemetry.addData("OmegaSpeedAuto", 0);

    }

    public void resetOdmetry(Pose2d rst){
        odometry.reset(new BTPose2d(rst));

        gyro.reset();
        odometry.reset(new BTPose2d(rst));

    }
    public void setMotors(double FL, double FR, double BL, double BR) {
        double compensation = 12.0/voltage_sensor.getVoltage();
        motor_FR.set(compensation*applyFeedFoward(chassisFR.ks,chassisFR.kv, FR));
        motor_FL.set(compensation*applyFeedFoward(chassisFL.ks,chassisFL.kv, FL));
        motor_BR.set(compensation*applyFeedFoward(chassisBR.ks,chassisBR.kv, BR));
        motor_BL.set(compensation*applyFeedFoward(chassisBL.ks,chassisBL.kv, BL));
        dashboardTelemetry.addData("compensation", compensation);
    }
    public double applyFeedFoward(double ks, double kv, double velocity){

        double s=velocity<0.01? 0: ks*Math.signum(velocity);// this is from kookybotz
        return s + kv * velocity;
    }

    ElapsedTime driveTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

     public boolean isFirstTime = true;

    //x is front facing, y is
    public BTCommand drive(DoubleSupplier frontVel, DoubleSupplier sidewayVel, DoubleSupplier retaliation) {
        return new RunCommand(() -> {
            if (isFirstTime) {
                driveTimer.reset();
                isFirstTime = false;

            }
            dashboardTelemetry.addData("drive: ", driveTimer.time());
            drive(frontVel.getAsDouble(), sidewayVel.getAsDouble(), retaliation.getAsDouble());
        }, this);

    }

    public BTCommand fieldRelativeDrive(DoubleSupplier frontVel, DoubleSupplier sidewayVel, DoubleSupplier rotation) {
        return new RunCommand(() -> {

            BTTranslation2d vector = new BTTranslation2d(sidewayVel.getAsDouble(), frontVel.getAsDouble());
            BTTranslation2d rotated = vector.rotateBy(BTRotation2d.fromDegrees(gyro.getHeading()));
            drive(rotated.getY(), rotated.getX(),  rotation.getAsDouble());
        }, this);
    }


    public Command stopMotor() {
        return new InstantCommand(()->{
            setMotors(0,0,0,0);
        });
    }


    @Override
    public void periodic() {
        m_pidcontroller.setPIDF(kp, ki, kd, kff);
        m_rotationpid.setPID(rkp,rki,rkd);
        m_rotationpid.setTolerance(tolerance);
        m_rotationpid.setIzone(rotIzone);
        odometry.updatePose();//todo: uncomment when starting to use odometry
        m_pidX.setPID(Xkp,Xki,Xkd);

        calcVA();

        dashboardTelemetry.addData("pose y: ", odometry.getPose().getY());
//        dashboardTelemetry.addData("pose gyro angle: ", gyro.getHeading());
        dashboardTelemetry.addData("pose odometry angle: ", odometry.getPose().getRotation().getDegrees());
        dashboardTelemetry.addData("pose x:", odometry.getPose().getX());

        dashboardTelemetry.addData("left encoder", m_leftEncoder.getPosition());
        dashboardTelemetry.addData("center encoder", m_centerEncoder.getPosition());
        dashboardTelemetry.addData("right encoder", m_rightEncoder.getPosition());

        dashboardTelemetry.addData("motor_BL",motor_BL.getVelocity());
        dashboardTelemetry.addData("motor_BR",motor_BR.getVelocity());
        dashboardTelemetry.addData("motor_FR",motor_FR.getVelocity());
        dashboardTelemetry.addData("motor_FL",motor_FL.getVelocity());

        dashboardTelemetry.addData("motor_BL current",motor_BL.motorEx.getCurrent(CurrentUnit.AMPS));
        dashboardTelemetry.addData("motor_BR current",motor_BR.motorEx.getCurrent(CurrentUnit.AMPS));
        dashboardTelemetry.addData("motor_FR current",motor_FR.motorEx.getCurrent(CurrentUnit.AMPS));
        dashboardTelemetry.addData("motor_FL current",motor_FL.motorEx.getCurrent(CurrentUnit.AMPS));

        RobotXAcc = odometry.getAcceleration();
        RobotContainer.armAccAdjustment = RobotXAcc;
        dashboardTelemetry.addData("RobotXAcc", RobotXAcc);
        dashboardTelemetry.addData("robotVolt", voltage_sensor.getVoltage());
        dashboardTelemetry.update();
        if (useTune==1) {
            setMotors(ChassisPower, ChassisPower, ChassisPower, ChassisPower);
        }
    }@Config
    public static class Tune{
        public static double useTune=0;
        public static double ChassisPower=0;
    }

    public void calcVA() {
        double dt = time.time() - prevTime;
        velocity = odometry.getPose().minus(prevPos).times(1 / dt);
        acceleration = velocity.minus(prevVelocity).times(1 / dt);


        prevPos = odometry.getPose();
        prevVelocity = velocity;
        prevAcceleration = acceleration;
        prevTime = time.time();

    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }

    public double metersFormTicks(int ticks) {
        return (ticks / (double) tickPerRevolution) * (2 * odometryWheelRadius * Math.PI);
    }

    public void chassisSpeedDrive(ChassisSpeeds chassisSpeeds){

        drive(chassisSpeeds.vyMetersPerSecond,chassisSpeeds.vxMetersPerSecond,chassisSpeeds.omegaRadiansPerSecond);

    }
    private void drive(double frontVel, double sidewayVel, double rotation) {
        double leftFrontPower  = frontVel + sidewayVel + rotation;
        double rightFrontPower = frontVel - sidewayVel - rotation;
        double leftBackPower   = frontVel - sidewayVel + rotation;
        double rightBackPower  = frontVel + sidewayVel - rotation;
//        dashboardTelemetry.addData("front vel in drive",frontVel);
//        dashboardTelemetry.addData("side vel in drive",sidewayVel);
//        dashboardTelemetry.addData("rot vel in drive",rotation);
//        double r = Math.hypot(rotation, sidewayVel);
//        double robotAngle = Math.atan2(rotation, sidewayVel) - Math.PI / 4;//shifts by 90 degrees so that 0 is to the right
//        double rightX = frontVel;
//        final double v1 = r * Math.cos(robotAngle) + rightX;
//        final double v2 = r * Math.sin(robotAngle) - rightX;
//        final double v3 = r * Math.sin(robotAngle) + rightX;
//        final double v4 = r * Math.cos(robotAngle) - rightX;
        setMotors(leftFrontPower, rightFrontPower, leftBackPower,rightBackPower);
    }

    public Pose2d getPosition() {
        return odometry.getPose();
    }

    public Command goToDegrees(double desiredAngle){
        return new InstantCommand(()->m_rotationpid.setSetpoint(desiredAngle)).andThen(new RunCommand(()->drive(0,0,m_rotationpid.calculate(odometry.getPose().getRotation().getDegrees()))));//-90

    }

   public Command goToX(double desiredX){
       return new InstantCommand(()->m_pidX.setSetpoint(desiredX)).andThen(new RunCommand(()->drive(m_pidX.calculate(odometry.getPose().getX()),0,0)));
   }

public Command goToY(double desiredY){
       return new InstantCommand(()->m_pidY.setSetpoint(desiredY)).andThen((new RunCommand(()->drive(0,m_pidY.calculate(odometry.getPose().getY()),0))));
   }

}



