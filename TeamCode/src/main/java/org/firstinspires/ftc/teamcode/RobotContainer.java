package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.BUMPER_LEFT;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.BUMPER_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.BUTTON_DOWN;
import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.BUTTON_LEFT;
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

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
//import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.climb;
import org.firstinspires.ftc.teamcode.subsystems.plane;
import org.firstinspires.ftc.teamcode.utils.BTController;
import org.firstinspires.ftc.teamcode.utils.Util;


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
    public static double armAccAdjustment = 0;


    public RobotContainer(HardwareMap map, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        armM1encoderR = new MotorEx(map, "ArmM1encoderR");//3
        armM2encoderL = new MotorEx(map, "ArmM2encoderL");//0
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        m_controller = new BTController(gamepad1);
        m_controller2 = new BTController(gamepad2);

        m_gripper = new Gripper(map, telemetry);
        m_chassis = new Chassis(map, telemetry, armM2encoderL.encoder, armM1encoderR.encoder);
        m_plane = new plane(map, telemetry);
        m_climb = new climb(map, telemetry);
        m_arm = new Arm(map, telemetry, armM2encoderL, armM1encoderR);


        bindCommands();
    }

    //bind commands to trigger
    public void bindCommands() {

        m_controller.assignCommand(m_chassis.fieldRelativeDrive(
                        () -> -m_controller.left_y.getAsDouble(),
                        m_controller.left_x,
                        () -> m_controller.right_trigger.getAsDouble() - m_controller.left_trigger.getAsDouble()),
                true, LEFT_X, LEFT_Y, LEFT_TRIGGER, RIGHT_TRIGGER).whenInactive(m_chassis.stopMotor());
        m_controller.assignCommand(m_climb.climb_manual(() -> -m_controller.right_x.getAsDouble()), true, RIGHT_X).whenInactive(m_climb.climb_manual(() -> 0));


//        m_controller2.assignCommand(m_arm.armMoveManual(m_controller2.left_y,()->m_controller2.right_trigger.getAsDouble()-m_controller2.left_trigger.getAsDouble()),true,RIGHT_TRIGGER,RIGHT_Y,LEFT_Y,LEFT_TRIGGER).whenInactive(m_arm.stopManual());
        m_controller2.assignCommand(m_arm.armMoveDriver(() -> -m_controller2.left_y.getAsDouble() * 0.7, () -> -m_controller2.right_x.getAsDouble() * 0.7), true, RIGHT_X, LEFT_Y).whenInactive(m_arm.stopAdjust());
//        m_controller2.assignCommand(m_arm.setState(DROP),true,DPAD_UP);
//        m_controller2.assignCommand(m_arm.setState(MIDDLE),true,DPAD_LEFT);
//        m_controller2.assignCommand(m_arm.setState(idle),true,DPAD_RIGHT);
//        m_controller2.assignCommand(m_arm.setState(PICKUP),true,DPAD_DOWN);
        m_controller2.assignCommand(m_gripper.toggleGripper1(), false, BUMPER_RIGHT);
        m_controller2.assignCommand(m_gripper.toggleGripper0(), false, BUMPER_LEFT);
        m_controller2.assignCommand(m_arm.toggleFF(), false, BUTTON_RIGHT);
        m_controller2.assignCommand(m_plane.shootPlane(), true, BUTTON_DOWN);
        m_controller2.assignCommand(m_plane.resetPlane(), true, BUTTON_UP);
        m_controller2.assignCommand(m_arm.setHighScore(), false, DPAD_UP);
        m_controller2.assignCommand(m_arm.setScore(), false, DPAD_DOWN);
        m_controller2.assignCommand(m_arm.setIdle(), false, DPAD_LEFT);
        m_controller2.assignCommand(m_arm.setMiddle(), false, BUTTON_LEFT);
        m_controller2.assignCommand(m_arm.setPickup(), false, DPAD_RIGHT);

    }

    public Command FarBlueAuto() {
        return new SequentialCommandGroup(

                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(5000),
                m_chassis.fieldRelativeDrive(() -> 0.9, () -> 0, () -> 0).withTimeout(100),
                m_chassis.fieldRelativeDrive(() -> 0, () -> -0.9, () -> 0).withTimeout(3000),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(1)

        );
    }

    public Command CloseBlueAuto() {
        return new SequentialCommandGroup(
                m_chassis.fieldRelativeDrive(() -> 0, () -> -0.9, () -> 0).withTimeout(1700),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(1)

        );
    }

    public Command TestAuto() {
        return new SequentialCommandGroup(
                m_chassis.fieldRelativeDrive(() -> 0, () -> -0.9, () -> 0).withTimeout(500),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(1)

        );
    }

    public Command FarRedAuto() {
        return new SequentialCommandGroup(
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(5000),
                m_chassis.fieldRelativeDrive(() -> -0.9, () -> 0, () -> 0).withTimeout(100),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0.9, () -> 0).withTimeout(3000),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(1)
        );
    }

    public Command CloseRedAuto() {
        return new SequentialCommandGroup(
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0.9, () -> 0).withTimeout(1700),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(1)
        );
    }

    public Command testPath() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(m_gripper.closeGripper0(),m_gripper.closeGripper1()),
                m_chassis.fieldRelativeDrive(() -> 0.9, () -> 0, () -> 0).withTimeout(1000).andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0.5).withTimeout(1000).andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(() -> -0.9, () -> 0, () -> 0).withTimeout(1000).andThen(m_chassis.stopMotor()),
                new WaitCommand(1000),
                m_arm.turnOnFF(),
                m_arm.setScore(),
                new ParallelCommandGroup(m_gripper.openGripper1(), m_gripper.openGripper0())
                );
    }
    public Command testPath2(){
        return new SequentialCommandGroup(
                m_gripper.closeGripper1(),
                m_gripper.closeGripper0(),
                m_chassis.fieldRelativeDrive(0.9,0,0).withTimeout(600),
                new WaitCommand(200),
                m_chassis.fieldRelativeDrive(0,0,0.5).withTimeout(300),
                new WaitCommand(100),
                m_chassis.fieldRelativeDrive(0,0.9,0).withTimeout(2000),
                new WaitCommand(200),
                m_arm.turnOnFF(),
                m_arm.setScore(),
                m_gripper.openGripper1(),
                m_arm.setIdle()

        );
    }
}