package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.*;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.*;

import android.graphics.Point;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.FollowPath;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climb;
import org.firstinspires.ftc.teamcode.subsystems.plane;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BTController;
import org.firstinspires.ftc.teamcode.utils.RunCommand;
import org.firstinspires.ftc.teamcode.utils.TrajectoryFactory;


public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {
    Chassis m_chassis;
    BTController m_controller;
    BTController m_controller2;
    Gripper m_gripper;
    plane m_plane;
    climb m_climb;
    Arm m_arm;
    MotorEx armM2encoderL;
    MotorEx armM1encoderR;

    public RobotContainer(HardwareMap map, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        armM1encoderR = new MotorEx(map, "ArmM1encoderR");//3
        armM2encoderL = new MotorEx(map, "ArmM2encoderL");//0
        m_controller = new BTController(gamepad1);
        m_controller2 = new BTController(gamepad2);

        m_gripper = new Gripper(map,telemetry);
        m_chassis = new Chassis(map, telemetry, armM2encoderL.encoder, armM1encoderR.encoder);
//        m_plane = new plane(map, telemetry);
        m_climb = new climb(map, telemetry);
        m_arm= new Arm(map, telemetry,armM2encoderL,armM1encoderR);


        bindCommands();
    }

    //bind commands to trigger
    public void bindCommands() {

        m_controller.assignCommand(m_chassis.fieldRelativeDrive(
                        () -> -m_controller.left_y.getAsDouble(),
                        m_controller.left_x,
                        () -> -m_controller.left_trigger.getAsDouble() + m_controller.right_trigger.getAsDouble()),
                true, LEFT_X, LEFT_Y, LEFT_TRIGGER, RIGHT_TRIGGER).whenInactive(m_chassis.stopMotor());

        m_controller2.assignCommand(m_arm.armMoveManual(()->Math.pow(m_controller2.right_y.getAsDouble(),3),()->Math.pow(m_controller2.left_y.getAsDouble(),3),()-> Math.pow(-m_controller2.left_trigger.getAsDouble()
                + m_controller2.right_trigger.getAsDouble(),3)),false,RIGHT_TRIGGER,LEFT_TRIGGER,LEFT_Y,RIGHT_Y);

        m_controller.assignCommand(m_climb.climb_manual(m_controller.right_y), true, RIGHT_Y).whenInactive(m_arm.stopManual());
        m_controller2.assignCommand(m_gripper.closeGripper(),false,BUMPER_LEFT);
        m_controller2.assignCommand(m_gripper.openGripper(),false,BUTTON_RIGHT);
//        m_controller2.assignCommand(m_plane.shootPlane(),false,DPAD_RIGHT);


    }

    public Command AutonomousCommand() {
        return null;
    }

    ;

}
