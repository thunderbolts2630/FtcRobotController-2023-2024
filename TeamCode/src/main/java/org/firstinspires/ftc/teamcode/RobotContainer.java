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
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
//import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.PixelDetection;
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
    public Arm m_arm;
    MotorEx armM2encoderL;
    MotorEx armM1encoderR;
    Gamepad gamepad1;
    Gamepad gamepad2;
    VoltageSensor voltage_sensor;
    public PixelDetection m_pixelDetection;
    public static double armAccAdjustment = 0;
    public static RobotContainer m_instance;


    public RobotContainer(HardwareMap map, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        armM1encoderR = new MotorEx(map, "ArmM1encoderR");//3
        armM2encoderL = new MotorEx(map, "ArmM2encoderC");//0
        voltage_sensor =  map.voltageSensor.iterator().next();

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        m_controller = new BTController(gamepad1);
        m_controller2 = new BTController(gamepad2);

        m_gripper = new Gripper(map, telemetry);
        m_chassis = new Chassis(map, telemetry, armM2encoderL.encoder, armM1encoderR.encoder,voltage_sensor);
        m_plane = new plane(map, telemetry);
        m_climb = new climb(map, telemetry);
        m_arm = new Arm(map, telemetry, armM2encoderL, armM1encoderR,voltage_sensor);
        m_pixelDetection=new PixelDetection(map,telemetry);

        oneDriver();
    }

    //bind commands to trigger
    public void oneDriver(){

        m_controller.assignCommand(m_chassis.fieldRelativeDrive(
                        () -> -m_controller.left_y.getAsDouble(),
                        m_controller.left_x,
                        () -> m_controller.right_trigger.getAsDouble() - m_controller.left_trigger.getAsDouble()),
                true, LEFT_X, LEFT_Y, LEFT_TRIGGER, RIGHT_TRIGGER).whenInactive(m_chassis.stopMotor());
        m_controller.assignCommand(m_climb.climb_manual(() -> -m_controller.right_x.getAsDouble()), true, RIGHT_X).whenInactive(m_climb.climb_manual(() -> 0));


        m_controller.assignCommand(m_gripper.toggleGripper0(), false, BUMPER_RIGHT);
        m_controller.assignCommand(m_gripper.toggleGripper1(), false, BUMPER_LEFT);

        m_controller.assignCommand(m_plane.shootPlane(), true, BUTTON_DOWN);
        m_controller.assignCommand(m_plane.resetPlane(), true, BUTTON_UP);

        m_controller.assignCommand(m_arm.toggleFF(), false, BUTTON_RIGHT);


        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setHighScore()), false, DPAD_UP);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setScore()), false, DPAD_DOWN);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setIdle()), false, DPAD_LEFT);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setLowScore()), false, BUTTON_LEFT);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setPickup()), false, DPAD_RIGHT);

    }

    public Command FarBlueAutoSimple() {
        return new SequentialCommandGroup(

                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(5000),
                m_chassis.fieldRelativeDrive(() -> 0.9, () -> 0, () -> 0).withTimeout(100),
                m_chassis.fieldRelativeDrive(() -> 0, () -> -0.9, () -> 0).withTimeout(3000),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(1)

        );
    }

    public Command CloseBlueAutoSimple() {
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

    public Command FarRedAutoSimple() {
        return new SequentialCommandGroup(
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(5000),
                m_chassis.fieldRelativeDrive(() -> -0.9, () -> 0, () -> 0).withTimeout(100),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0.9, () -> 0).withTimeout(3000),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(1)
        );
    }

    public Command CloseRedAutoSimple() {
        return new SequentialCommandGroup(
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0.9, () -> 0).withTimeout(1700),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0).withTimeout(1)
        );
    }

    public Command testPath() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(m_gripper.closeGripper0(), m_gripper.closeGripper1()),
                m_chassis.fieldRelativeDrive(() -> 0.9, () -> 0, () -> 0).withTimeout(1000).andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(() -> 0, () -> 0, () -> 0.5).withTimeout(1000).andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(() -> -0.9, () -> 0, () -> 0).withTimeout(1000).andThen(m_chassis.stopMotor()),
                new WaitCommand(1000),
                m_arm.turnOnFF(),
                m_arm.setScore(),
                new ParallelCommandGroup(m_gripper.openGripper1(), m_gripper.openGripper0())
        );
    }

    public Command centerCloseBluePath() {
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, -0.6, 0).andThen(new WaitCommand(820)),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(850)),
                m_chassis.fieldRelativeDrive(0,0,90).andThen(new WaitCommand(500))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(500)),
                m_gripper.openGripper1(),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0.6, 90).andThen(new WaitCommand(500)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive( 0.6,0,90).andThen(new WaitCommand(850)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0,0,-85).andThen(new WaitCommand(700))
                        .andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(-0.4,0,-90).andThen(new WaitCommand(80)),
                m_chassis.stopMotor().andThen(new WaitCommand(300)),
                m_chassis.fieldRelativeDrive(0,-0.5,-90).andThen(new WaitCommand(120)),
                m_chassis.stopMotor().andThen(new WaitCommand(300)),
                m_arm.setLowScore().andThen(new WaitCommand(1500)),

                m_chassis.fieldRelativeDrive(0,0.6,-90).andThen(new WaitCommand(180)),
                m_chassis.stopMotor().andThen(new WaitCommand(1500)),
                m_gripper.openGripper0().andThen(new WaitCommand(400)),
                m_chassis.fieldRelativeDrive(0,-0.6,-90).andThen(new WaitCommand(260))
                        .andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(-0.5,0,-90).andThen(new WaitCommand(50)),
                m_chassis.stopMotor(),
                new WaitCommand(300),
                m_gripper.closeGripper0(),

                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0.6,-90).andThen(new WaitCommand(200)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(4000)),
                m_chassis.stopMotor()
        );
    }



    public Command leftCloseBluePath() {
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, -0.6, 0).andThen(new WaitCommand(870)),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(450))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(600)),
                m_gripper.openGripper1(),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(-0.6, 0, 0).andThen(new WaitCommand(50)),
                m_chassis.fieldRelativeDrive(0, 0, -90).andThen(new WaitCommand(450)),
                m_chassis.fieldRelativeDrive(0, 0.6, -90).andThen(new WaitCommand(370))
                        .andThen(m_chassis.stopMotor()),
                        m_chassis.fieldRelativeDrive(-0.6,0,-90).andThen(new WaitCommand(500))
                        .andThen(m_chassis.stopMotor()),

                m_arm.setLowScore().andThen(new WaitCommand(1000)),


                m_gripper.openGripper0().andThen(new WaitCommand(400)),
                m_chassis.fieldRelativeDrive(0,-0.6, -90).andThen(new WaitCommand(100)),
                m_chassis.stopMotor(),
                m_gripper.closeGripper0(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(4000)),
                m_chassis.stopMotor()
        );
    }
    public Command rightCloseBluePath() {
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, -0.6, 0).andThen(new WaitCommand(530)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(510))
                        .andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(0, 0, -90).andThen(new WaitCommand(400)),
                m_arm.turnOnFF(),
                m_arm.setPickup().andThen(new WaitCommand(600)),
                m_gripper.openGripper1(),
                m_chassis.fieldRelativeDrive(0,0.6,-90).andThen(new WaitCommand(200)),
                m_chassis.stopMotor(),
                m_gripper.closeGripper1(),
                m_arm.setMiddle(),
                m_chassis.fieldRelativeDrive(0,0.6,-90).andThen(new WaitCommand(420)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(-0.6,0,-90).andThen(new WaitCommand(480)),
                m_chassis.stopMotor(),
                m_arm.setLowScore(),
                m_gripper.openGripper0().andThen(new WaitCommand(400)),
                m_chassis.fieldRelativeDrive(0,-0.6,-90).andThen(new WaitCommand(260))
                        .andThen(m_chassis.stopMotor()),
                m_gripper.closeGripper0(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0.6,-90).andThen(new WaitCommand(200)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(4000)),
                m_chassis.stopMotor()
        );
    }

        public Command leftBlueFarPath(){
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, 0.6, 0).andThen(new WaitCommand(240)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(540))
                        .andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(0, 0, -90).andThen(new WaitCommand(600)),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(600)),
                m_gripper.openGripper1(),
                m_gripper.closeGripper1(),
                m_arm.setIdle().andThen(new WaitCommand(600)),
                m_chassis.fieldRelativeDrive(0,0.6, -90).andThen(new WaitCommand(70)),
                m_chassis.fieldRelativeDrive(-0.6,0,-90).andThen(new WaitCommand(520)),
                m_chassis.fieldRelativeDrive(0,-0.6,-90).andThen(new WaitCommand(1000)),
                m_chassis.fieldRelativeDrive(0.6,0,-90).andThen(new WaitCommand(500)),
                m_chassis.stopMotor(),
                m_arm.setLowScore(),
                m_gripper.openGripper0().andThen(new WaitCommand(500)),
                m_gripper.closeGripper0(),
                m_arm.setIdle()


                );

        }

        public Command centerFarBluePath(){
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, 0.6, 0).andThen(new WaitCommand(800)),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(900)),
                m_chassis.fieldRelativeDrive(0,0,-90).andThen(new WaitCommand(500))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(500)),
                m_gripper.openGripper1(),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(-0.6,0,-90).andThen(new WaitCommand(860)),
                m_chassis.fieldRelativeDrive(0,-0.6,-90).andThen(new WaitCommand(1800)),
                m_chassis.fieldRelativeDrive(0.6,0,-90).andThen(new WaitCommand(500)),
                m_chassis.stopMotor(),
                m_arm.setLowScore(),
                m_gripper.openGripper0().andThen(new WaitCommand(500)),
                m_gripper.closeGripper0(),
                m_arm.setIdle()


        );
        }

        public Command rightFarBluePath(){
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, 0.6, 0).andThen(new WaitCommand(750)),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(520))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(600)),
                m_gripper.openGripper1(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(-0.6,0,0).andThen(new WaitCommand(500)),
                m_chassis.fieldRelativeDrive(0,0,-90).andThen(new WaitCommand(600)),
                m_chassis.fieldRelativeDrive(-0.6,0,-90).andThen(new WaitCommand(1500)),
                m_chassis.fieldRelativeDrive(0.6,0,-90).andThen(new WaitCommand(460)),
                m_chassis.stopMotor(),
                m_arm.setLowScore(),
                m_gripper.openGripper0().andThen(new WaitCommand(500)),
                m_gripper.closeGripper0(),
                m_arm.setIdle()

        );
        }


}