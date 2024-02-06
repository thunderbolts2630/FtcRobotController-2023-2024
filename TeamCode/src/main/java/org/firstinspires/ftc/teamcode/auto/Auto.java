package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.util.Supplier;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.function.Consumer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.utils.HolonomicOdometry;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.geometry.BTPose2d;

@Autonomous(name="Auto", group = "Linear Opmode")
public class Auto extends LinearOpMode {
    FollowPath path; //a
    ElapsedTime time;
    List<Trajectory.State> states; //?
    HolonomicOdometry pose; //a
    PIDController xController; //a
    PIDController yController; //a
    ProfiledPIDController thetaController; //a
    Consumer<ChassisSpeeds> consumer; //?
    Rotation2d rotation2d; //a
    Chassis m_chassis; //a
    public Auto(Chassis chassis) {
        super();
        time.startTime();
        m_chassis = chassis;
        pose = new HolonomicOdometry(Constants.ChassisConstants.TRACKWIDTH, Constants.ChassisConstants.WHEEL_OFFSET);
        xController = new PIDController(Constants.ChassisConstants.PIDConstants.kpX,Constants.ChassisConstants.PIDConstants.kiX,Constants.ChassisConstants.PIDConstants.kdX);
        yController = new PIDController(Constants.ChassisConstants.PIDConstants.kpY,Constants.ChassisConstants.PIDConstants.kiY,Constants.ChassisConstants.PIDConstants.kdY);
        thetaController = new ProfiledPIDController(Constants.ChassisConstants.PIDConstants.kpT,Constants.ChassisConstants.PIDConstants.kiT,Constants.ChassisConstants.PIDConstants.kdT, Constants.ChassisConstants.PIDConstants.kcT);
        rotation2d = new Rotation2d(0);
        consumer = chassisSpeeds -> {

        };
        states = new ArrayList<Trajectory.State>();

        this.path = new FollowPath(new Trajectory(states), Supplier<new Pose2d(pose.getPose().getTranslation(), pose.getPose().getRotation())>, xController, yController, thetaController,
                rotation2d, consumer, m_chassis);
    }

    @Override
    public void runOpMode() {
    waitForStart();
    }
}
