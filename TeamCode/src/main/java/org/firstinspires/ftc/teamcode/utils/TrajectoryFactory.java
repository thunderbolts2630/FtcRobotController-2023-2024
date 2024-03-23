package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.Arrays;

public class TrajectoryFactory {
    public static Trajectory generate(TrajectoryConfig config,Pose2d... waypoints){
        if( waypoints.length<2){
            throw new RuntimeException("too few waypoints to generate path");
        }
        return TrajectoryGenerator.generateTrajectory(Arrays.asList(waypoints),config);

    }
    private static final TrajectoryConfig FrontConfig =
            new TrajectoryConfig(Constants.ChassisConstants.RobotMaxVelFront,
                    Constants.ChassisConstants.RobotMaxAccFront)
                    .setKinematics(Constants.ChassisConstants.kinematics);

    public static final Trajectory _forward = generate(FrontConfig,
            new Pose2d(0,0, Rotation2d.fromDegrees(0)),
            new Pose2d(0,1,Rotation2d.fromDegrees(0))
    );
    public static final Trajectory _90degrees = generate(FrontConfig,
            new Pose2d(0,0, Rotation2d.fromDegrees(0)),
            new Pose2d(1,1,Rotation2d.fromDegrees(90))
    );


}



