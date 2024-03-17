package org.firstinspires.ftc.teamcode.auto.Path;


import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.ChassisFeedForward.*;
import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.*;
import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.PIDConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.*;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BT.BTHolonomicDriveController;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class FollowPath extends BTCommand {
    public static class FellowPathConfig{
        Supplier<Pose2d> pose;
        BTHolonomicDriveController controller;
        Supplier<Rotation2d> desiredRotation;
        Consumer<ChassisSpeeds> outputChassisSpeeds;
        MecanumDriveKinematics kinematics;
        Consumer<Pose2d> resetOdometry;
        Subsystem requirments;

        public FellowPathConfig(Supplier<Pose2d> pose, BTHolonomicDriveController controller, Supplier<Rotation2d> desiredRotation, Consumer<ChassisSpeeds> outputChassisSpeeds, Consumer<Pose2d> resetOdometry, Subsystem requirments) {
            this.pose = pose;
            this.controller = controller;
            this.desiredRotation = desiredRotation;
            this.outputChassisSpeeds = outputChassisSpeeds;
            this.resetOdometry = resetOdometry;
            this.requirments = requirments;
        }
    }
    private final ElapsedTime m_timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final BTHolonomicDriveController m_controller;
    private final Consumer<ChassisSpeeds> m_outputChassisSpeeds;
    private final Supplier<Rotation2d> m_desiredRotation;
    private final Consumer<Pose2d> m_resetOdometry;
    private Telemetry dashboard =FtcDashboard.getInstance().getTelemetry();


    public FollowPath(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Supplier<Rotation2d> desiredRotation,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            MecanumDriveKinematics kinematics,
            Consumer<Pose2d> resetOdometry,
            Subsystem... requirments) {
        this(
                trajectory,
                pose,
                new BTHolonomicDriveController(
                        xController,
                        yController,
                        thetaController),
                desiredRotation,
                outputChassisSpeeds,
                resetOdometry,
                requirments
                );
    }

    public FollowPath(
            Trajectory trajectory,
            FollowPath.FellowPathConfig config
    ){
        this(trajectory,
                config.pose,
                config.controller,
                config.desiredRotation,
                config.outputChassisSpeeds,
                config.resetOdometry,
                config.requirments
        );
    }

    public FollowPath(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            BTHolonomicDriveController controller,
            Supplier<Rotation2d> desiredRotation,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            Consumer<Pose2d> resetOdometry,
            Subsystem... requirments) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_controller = controller;
        m_desiredRotation = desiredRotation;
        m_outputChassisSpeeds = outputChassisSpeeds;
        this.m_resetOdometry= resetOdometry;
        addRequirements(requirments);
        dashboard.addData("DfrontVelocity: ", 0);
        dashboard.addData("DsideVelocity: ", 0);
        dashboard.addData("DomegaVelocity: ", 0);
        dashboard.addData("followPath time", m_timer.time());

    }

    @Override
    public void initialize() {
        m_timer.reset();


        //        m_controller.m_thetaController.setConstraints(
//                new TrapezoidProfile.Constraints(
//                        in.get("maxV"),
//                        Math.pow(in.get("maxV"),2)
//                        0,0
//                ));
//        m_controller.m_thetaController.setPID(
//                in.get("kP"),
//                in.get("kI"),
//                in.get("kD")
//        );
        m_resetOdometry.accept(m_trajectory.sample(0).poseMeters);
        m_controller.reset();
        m_controller.m_thetaController.setPID(PIDTheta.kpT,PIDTheta.kiT,PIDTheta.kdT);
        m_controller.m_thetaController.setConstraints(new TrapezoidProfile.Constraints(PIDTheta.maxVelocity,PIDTheta.maxAcceleration));
        m_controller.m_xController.setPID(PIDFront.kpX,PIDFront.kiX,PIDFront.kdX);
        m_controller.m_yController.setPID(PIDSide.kpY,PIDSide.kiY,PIDSide.kdY);

    }




    @Override
    public void execute() {
        double curTime = m_timer.time();
        dashboard.addData("followPath time", curTime);

        Trajectory.State desiredState = m_trajectory.sample(curTime);
        dashboard.addData("state velocity", desiredState.velocityMetersPerSecond);
        if (m_desiredRotation.get() == null){
            ChassisSpeeds targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, m_trajectory.getStates().get(m_trajectory.getStates().size() - 1).poseMeters.getRotation());
            dashboard.addData("DfrontVelocity: ", targetChassisSpeeds.vyMetersPerSecond);
            dashboard.addData("DsideVelocity: ", targetChassisSpeeds.vxMetersPerSecond);
            dashboard.addData("DomegaVelocity: ", targetChassisSpeeds.omegaRadiansPerSecond);
            targetChassisSpeeds=addFeedForward(targetChassisSpeeds);
            m_outputChassisSpeeds.accept(targetChassisSpeeds);
            dashboard.update();


        } else {
            ChassisSpeeds targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
            targetChassisSpeeds= addFeedForward(targetChassisSpeeds);
            m_outputChassisSpeeds.accept(targetChassisSpeeds);
        }
    }

    private ChassisSpeeds addFeedForward(ChassisSpeeds chassisSpeeds){
        ChassisSpeeds afterFF= new ChassisSpeeds();
        afterFF.vyMetersPerSecond= chassisSpeeds.vyMetersPerSecond/RobotMaxVelFront;
        afterFF.vxMetersPerSecond=chassisSpeeds.vxMetersPerSecond/RobotMaxVelSide;
        afterFF.omegaRadiansPerSecond=chassisSpeeds.omegaRadiansPerSecond/robotThetaVelocityMax;

        afterFF.vxMetersPerSecond=chassisSpeeds.vxMetersPerSecond+calculateFF(afterFF.vxMetersPerSecond);
        return  afterFF;
    }
    private double calculateFF(double velocity) {
        return ffks * Math.signum(velocity) + ffkv * velocity ;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return m_timer.time() > (m_trajectory.getTotalTimeSeconds());
    }
}





