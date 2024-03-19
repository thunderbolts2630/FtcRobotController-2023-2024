package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
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

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Chassis implements Subsystem {
    public static boolean hasReset = false;

    private DoubleSupplier voltage_sensor;
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
    private DcMotorImplEx motor_FL,motor_FR,motor_BL,motor_BR;
    private RevIMU gyro;
    private BTPose2d m_postitionFromTag;
    private double maxVelocityX = 0;
    private double maxVelocityY = 0;
    private double maxVelocityTheta = 0;
    private double maxAccelerationX = 0;
    private double maxAccelerationTheta = 0;
    private double maxAccelerationY = 0;
    private Supplier<Integer> m_leftEncoder, m_centerEncoder, m_rightEncoder;
    public double RobotXAcc = 0;
    public int test = 0;

    ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private boolean auto = false;
    private double desiredXVel;
    private double desiredYVel;
    private double desiredAngle;


    public Chassis(HardwareMap map,
                   DcMotorImplEx motorFL, DcMotorImplEx motorFR, DcMotorImplEx motorBL, DcMotorImplEx motorBR,
                   Supplier<Integer> leftEncoder, Supplier<Integer> rightEncoder, Supplier<Integer> centerEncoder,
                   DoubleSupplier voltage_sensor) {
        this.map = map;
        motor_FL = motorFL;//1
        motor_FR = motorFR;//2
        motor_BL = motorBL;//0
        motor_BR = motorBR;//3
        this.voltage_sensor = voltage_sensor;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro = new RevIMU(map, "imu");
        gyro.init(parameters);
        if (!hasReset) {
            gyro.reset();
            hasReset = true;
        }
        gyro.invertGyro();
        motor_FR.setZeroPowerBehavior(BRAKE);
        motor_BR.setZeroPowerBehavior(BRAKE);
        motor_BL.setZeroPowerBehavior(BRAKE);
        motor_FL.setZeroPowerBehavior(BRAKE);
        m_leftEncoder = leftEncoder;
        m_centerEncoder = centerEncoder;
        m_rightEncoder = rightEncoder;
        odometry = new BTposeEstimator(
                () -> -metersFormTicks(m_leftEncoder.get()),
                () -> metersFormTicks(m_rightEncoder.get()),
                () -> metersFormTicks(m_centerEncoder.get()),
                () -> gyro.getHeading(),
                TRACKWIDTH, WHEEL_OFFSET);
        prevPos = odometry.getPose();
        time.reset();
        time.startTime();
        m_rotFF = new SimpleMotorFeedforward(rks, 1);
        prevTime = time.time();
        m_pidcontroller = new PIDFController(kp, ki, kd, kff);
        m_rotationpid = new PIDController(rkp, rki, rkd);
        m_rotationpid.enableContinuousInput(-180, 180);
        m_rotationpid.setTolerance(tolerance);
        register();
        dashboardTelemetry.addData("drive: ", 0);
        dashboardTelemetry.addData("frontVelocityAuto", 0);
        dashboardTelemetry.addData("sideVelocityAuto", 0);
        dashboardTelemetry.addData("OmegaSpeedAuto", 0);

    }

    public void resetOdmetry(Pose2d rst) {
        gyro.reset();
        odometry.reset(new BTPose2d(rst));

    }

    public void setMotors(double FL, double FR, double BL, double BR) {
        double compensation = 12.0 / voltage_sensor.getAsDouble();
        motor_FR.setPower(compensation * FR);
        motor_FL.setPower(compensation * FL);
        motor_BR.setPower(compensation * BR);
        motor_BL.setPower(compensation * BL);
        dashboardTelemetry.addData("compensation", compensation);
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

    public BTCommand fieldRelativeDrive(DoubleSupplier frontVel, DoubleSupplier sidewayVel, DoubleSupplier retaliation) {
        return new RunCommand(() -> {

            BTTranslation2d vector = new BTTranslation2d(sidewayVel.getAsDouble(), frontVel.getAsDouble());
            BTTranslation2d rotated = vector.rotateBy(BTRotation2d.fromDegrees(gyro.getHeading()));
            drive(rotated.getY(), rotated.getX(), retaliation.getAsDouble());
        }, this);
    }

    public Command fieldRelativeDrive(double frontVel, double sidewayVel, double retaliation) {
        return new InstantCommand(() -> {
            auto = true;
            BTTranslation2d vector = new BTTranslation2d(sidewayVel, frontVel);
            BTTranslation2d rotated = vector.rotateBy(BTRotation2d.fromDegrees(-gyro.getHeading()));
            desiredXVel = rotated.getY();
            desiredYVel = rotated.getX();
            desiredAngle = retaliation;
        }, this);
    }

    public Command stopMotor() {
        return new InstantCommand(() -> {
            auto = false;
            setMotors(0, 0, 0, 0);
        });
    }

    @Override
    public void periodic() {
        m_pidcontroller.setPIDF(kp, ki, kd, kff);
        m_rotationpid.setPID(rkp, rki, rkd);
        m_rotationpid.setTolerance(tolerance);
        m_rotationpid.setIzone(rotIzone);
        odometry.updatePose();
        calcVA();
        if (auto) {
            drive(desiredXVel, desiredYVel, m_rotFF.calculate(m_rotationpid.calculate(gyro.getHeading(), desiredAngle)));
        }
        dashboardTelemetry.addData("pose y: ", odometry.getPose().getY());
        dashboardTelemetry.addData("pose gyro angle: ", gyro.getHeading());
        dashboardTelemetry.addData("pose x:", odometry.getPose().getX());

        dashboardTelemetry.addData("FL velocity", motor_FL.getVelocity());
        dashboardTelemetry.addData("FR velocity", motor_FR.getVelocity());
        dashboardTelemetry.addData("BR velocity", motor_BR.getVelocity());
        dashboardTelemetry.addData("BL velocity", motor_BL.getVelocity());

        RobotXAcc = odometry.getAcceleration();
        RobotContainer.armAccAdjustment = RobotXAcc;
        dashboardTelemetry.addData("RobotXAcc", RobotXAcc);
        dashboardTelemetry.addData("robotVolt", voltage_sensor.getAsDouble());

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

    public void chassisSpeedDrive(ChassisSpeeds chassisSpeeds) {

        drive(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);

    }

    private void drive(double frontVel, double sidewayVel, double retaliation) {

        dashboardTelemetry.addData("front vel in drive", frontVel);
        dashboardTelemetry.addData("side vel in drive", sidewayVel);
        dashboardTelemetry.addData("rot vel in drive", retaliation);
        double r = Math.hypot(retaliation, sidewayVel);
        double robotAngle = Math.atan2(retaliation, sidewayVel) - Math.PI / 4;//shifts by 90 degrees so that 0 is to the right
        double rightX = frontVel;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        setMotors(v1, v2, v3, v4);
    }

    public Pose2d getPosition() {
        return odometry.getPose();
    }

    public Command goToDegrees(double degrees) {
        return new InstantCommand(() -> {
            m_rotationpid.setSetpoint(degrees);//-90
        });
    }

    public Command test() {
        return new InstantCommand(() -> test = 1);
    }

}



