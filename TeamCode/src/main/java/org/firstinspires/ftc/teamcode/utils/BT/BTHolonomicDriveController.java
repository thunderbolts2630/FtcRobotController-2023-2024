// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.utils.BT;


import com.arcrobotics.ftclib.geometry.*;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.utils.PID.*;


/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
public class BTHolonomicDriveController {
    private Pose2d m_poseError = new Pose2d();
    private Rotation2d m_rotationError = new Rotation2d();
    private Pose2d m_poseTolerance = new Pose2d();
    private boolean m_enabled = true;

    public final PIDController m_xController;
    public final PIDController m_yController;
    public final ProfiledPIDController m_thetaController;

    public boolean m_firstRun = true;

    /**
     * Constructs a holonomic drive controller.
     *
     * @param xController A PID Controller to respond to error in the field-relative x direction.
     * @param yController A PID Controller to respond to error in the field-relative y direction.
     * @param thetaController A profiled PID controller to respond to error in angle.
     */


    public BTHolonomicDriveController(
            PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
        m_thetaController.enableContinuousInput(0, 2*Math.PI);
    }

    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        final Translation2d eTranslate = m_poseError.getTranslation();
        final Rotation2d eRotate = m_rotationError;
        final Translation2d tolTranslate = m_poseTolerance.getTranslation();
        final Rotation2d tolRotate = m_poseTolerance.getRotation();
        return Math.abs(eTranslate.getX()) < tolTranslate.getX()
                && Math.abs(eTranslate.getY()) < tolTranslate.getY()
                && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }

    /**
     * Sets the pose error which is considered tolerance for use with atReference().
     *
     * @param tolerance The pose error which is tolerable.
     */
    public void setTolerance(Pose2d tolerance) {
        m_poseTolerance = tolerance;
    }

    public ProfiledPIDController getM_thetaController(){
        return m_thetaController;
    }


    /**
     * Returns the next output of the holonomic drive controller.
     *
     * @param currentPose The current pose.
     * @param poseRef The desired pose.
     * @param linearVelocityRefMeters The linear velocity reference.
     * @param angleRef The angular reference.
     * @return The next output of the holonomic drive controller.
     */
    public ChassisSpeeds calculate(
            Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, Rotation2d angleRef) {
        // If this is the first run, then we need to reset the theta controller to the current pose's
        // heading.
        if (m_firstRun) {
            m_thetaController.reset(currentPose.getRotation().getRadians());
            m_firstRun = false;
        }

        // Calculate feedforward velocities (field-relative).
        double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
        double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
        double thetaFF =
                m_thetaController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

        m_poseError = poseRef.relativeTo(currentPose);
        m_rotationError = angleRef.minus(currentPose.getRotation());

        if (!m_enabled) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
        }

        // Calculate feedback velocities (based on position error).
        double xFeedback = m_xController.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback = m_yController.calculate(currentPose.getY(), poseRef.getY());

        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    }

    /**
     * Returns the next output of the holonomic drive controller.
     *
     * @param currentPose The current pose.
     * @param desiredState The desired trajectory state.
     * @param angleRef The desired end-angle.
     * @return The next output of the holonomic drive controller.
     */
    public ChassisSpeeds calculate(
            Pose2d currentPose, Trajectory.State desiredState, Rotation2d angleRef) {
        return calculate(
                currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, angleRef);
    }

    /**
     * Enables and disables the controller for troubleshooting problems. When calculate() is called on
     * a disabled controller, only feedforward values are returned.
     *
     * @param enabled If the controller is enabled or not.
     */
    public void setEnabled(boolean enabled) {
        m_enabled = enabled;
    }
    public void reset(){
        m_firstRun = true;
        m_xController.reset();
        m_yController.reset();

    }


}