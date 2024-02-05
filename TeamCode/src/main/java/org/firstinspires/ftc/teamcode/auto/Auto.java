package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import java.util.List;
import java.util.function.Consumer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.utils.HolonomicOdometry;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.geometry.BTPose2d;

@Autonomous(name="Auto", group = "Linear Opmode")
public class Auto extends LinearOpMode {
    FollowPath path;
    List<Trajectory.State> states;
    HolonomicOdometry pose;
    PIDController xController;
    PIDController yController;
    ProfiledPIDController thetaController;
    Consumer<ChassisSpeeds> consumer;
    Rotation2d rotation2d = new Rotation2d(0);
    Chassis m_chassis;
    public Auto() {
        super();
        this.path = new FollowPath(new Trajectory(states), pose.getPose(), xController, yController, thetaController,
                rotation2d, consumer, m_chassis);
    }

    @Override
    public void runOpMode() {
    waitForStart();
    }
}
