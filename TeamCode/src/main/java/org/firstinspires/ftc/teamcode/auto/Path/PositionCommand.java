package org.firstinspires.ftc.teamcode.auto.Path;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

@Config
public class PositionCommand extends CommandBase {
    public Pose2d targetPose;

    public static double xP = 0.07;
    public static double xD = 0.012;

    public static double yP = 0.07;
    public static double yD = 0.012;

    public static double hP = 1;
    public static double hD = 0.045;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    public static double ALLOWED_TRANSLATIONAL_ERROR = 0.75;
    public static double ALLOWED_HEADING_ERROR = 0.02;


    private ElapsedTime timer;
    private ElapsedTime stable;

    public static double STABLE_MS = 250;
    public static double DEAD_MS = 2500;

    private final double  MAX_TRANSLATIONAL_SPEED = 0.5;
    private final double  MAX_ROTATIONAL_SPEED = 0.4;
    private final double K_STATIC = 1.85;

    Chassis drivetrain;
    public PositionCommand(Pose2d targetPose,Chassis drivetrain) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;

        xController.reset();
        yController.reset();
        hController.reset();
    }

    /**
     *
     */
    @Override
    public void execute() {
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        Pose2d robotPose = drivetrain.getPosition();

//        System.out.println("TARGET Pose2d " + targetPose);


        ChassisSpeeds powers = getPower(robotPose);
        drivetrain.drive(powers.vyMetersPerSecond,powers.vxMetersPerSecond,powers.omegaRadiansPerSecond);
    }

    @Override
    public boolean isFinished() {
        Pose2d robotPose = drivetrain.getPosition();
        Transform2d delta = targetPose.minus(robotPose);

        if (delta.getTranslation().getNorm() > ALLOWED_TRANSLATIONAL_ERROR
                || Math.abs(delta.getRotation().getRadians()) > ALLOWED_HEADING_ERROR) {
            stable.reset();
        }

        return timer.milliseconds() > DEAD_MS || stable.milliseconds() > STABLE_MS;
    }

    public ChassisSpeeds getPower(Pose2d robotPose) {
        double xPower = xController.calculate(robotPose.getX(), targetPose.getX());
        double yPower = yController.calculate(robotPose.getY(), targetPose.getY());
        double hPower = -hController.calculate(robotPose.getHeading(), targetPose.getHeading());

        double x_rotated = xPower * Math.cos(-robotPose.getHeading()) - yPower * Math.sin(-robotPose.getHeading());
        double y_rotated = xPower * Math.sin(-robotPose.getHeading()) + yPower * Math.cos(-robotPose.getHeading());

        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED / K_STATIC, MAX_TRANSLATIONAL_SPEED / K_STATIC);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);

        return new ChassisSpeeds(x_rotated * K_STATIC, y_rotated,hPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.resetOdmetry(new Pose2d());
    }
}
