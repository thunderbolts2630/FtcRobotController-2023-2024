package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.Positions.*;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.BUMPER_LEFT;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.BUMPER_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.BUTTON_DOWN;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.BUTTON_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.BUTTON_UP;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.DPAD_DOWN;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.DPAD_LEFT;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.DPAD_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.DPAD_UP;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.LEFT_TRIGGER;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.LEFT_X;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.LEFT_Y;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.RIGHT_X;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.RIGHT_Y;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.climb;
import org.firstinspires.ftc.teamcode.subsystems.plane;
import org.firstinspires.ftc.teamcode.utils.BTController;


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
    Gamepad gamepad1;
    Gamepad gamepad2;


    public RobotContainer(HardwareMap map, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        armM1encoderR = new MotorEx(map, "ArmM1encoderR");//3
        armM2encoderL = new MotorEx(map, "ArmM2encoderL");//0
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        m_controller = new BTController(gamepad1);
        m_controller2 = new BTController(gamepad2);

        m_gripper = new Gripper(map,telemetry);
        m_chassis = new Chassis(map, telemetry, armM2encoderL.encoder, armM1encoderR.encoder);
        m_plane = new plane(map, telemetry);
        m_climb = new climb(map, telemetry);
        m_arm= new Arm(map, telemetry,armM2encoderL,armM1encoderR);


        bindCommands();
    }

    //bind commands to trigger
    public void bindCommands() {

        m_controller.assignCommand(m_chassis.fieldRelativeDrive(
                        () -> -m_controller.left_y.getAsDouble(),
                        m_controller.left_x,
                         ()->m_controller.left_trigger.getAsDouble()-m_controller.right_trigger.getAsDouble()),
                true, LEFT_X, LEFT_Y, LEFT_TRIGGER,RIGHT_TRIGGER).whenInactive(m_chassis.stopMotor());
        m_controller.assignCommand(m_climb.climb_manual(m_controller.right_x), true, RIGHT_X).whenInactive(m_climb.climb_manual(()->0));


//        m_controller2.assignCommand(m_arm.armMoveManual(m_controller2.left_y,()->m_controller2.right_trigger.getAsDouble()-m_controller2.left_trigger.getAsDouble()),true,RIGHT_TRIGGER,RIGHT_Y,LEFT_Y,LEFT_TRIGGER).whenInactive(m_arm.stopManual());
        m_controller2.assignCommand(m_arm.armMoveManual(m_controller2.left_y,m_controller2.right_y),true,RIGHT_TRIGGER,RIGHT_Y,LEFT_Y,LEFT_TRIGGER).whenInactive(m_arm.stopManual());
        m_controller2.assignCommand(m_arm.setState(DROP),true,DPAD_UP);
        m_controller2.assignCommand(m_arm.setState(MIDDLE),true,DPAD_LEFT);
        m_controller2.assignCommand(m_arm.setState(idle),true,DPAD_RIGHT);
        m_controller2.assignCommand(m_arm.setState(PICKUP),true,DPAD_DOWN);
        m_controller2.assignCommand(m_gripper.closeGripper(),true,BUMPER_LEFT).whenInactive(m_gripper.stop());
        m_controller2.assignCommand(m_gripper.openGripper(),true,BUMPER_RIGHT).whenInactive(m_gripper.stop());
        m_controller2.assignCommand(m_plane.shootPlane(),true,BUTTON_DOWN);
        m_controller2.assignCommand(m_plane.resetPlane(),true,BUTTON_UP);


    }

    public Command AutonomousCommand() {
        return null;
    }

    ;

}
