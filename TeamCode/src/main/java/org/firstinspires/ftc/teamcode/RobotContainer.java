package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_LEFT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_DOWN;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_LEFT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_UP;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.DPAD_DOWN;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.DPAD_LEFT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.DPAD_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.*;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.LEFT_X;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.Path.FollowPath;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
//import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.HardwareCache;
import org.firstinspires.ftc.teamcode.subsystems.pixelDetector.PixelDetection;
import org.firstinspires.ftc.teamcode.subsystems.climb;
import org.firstinspires.ftc.teamcode.subsystems.plane;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;
import org.firstinspires.ftc.teamcode.utils.BT.BTHolonomicDriveController;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.utils.TrajectoryFactory;

import java.util.List;

import javax.annotation.Nullable;


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
        //enable bulk read
        List<LynxModule> allHubs = map.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        armM1encoderR = new MotorEx(map, "ArmM1encoderR");//3
        armM2encoderL = new MotorEx(map, "ArmM2encoderC");//0
        voltage_sensor =  map.voltageSensor.iterator().next();

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        m_controller = new BTController(gamepad1);
        m_controller2 = new BTController(gamepad2);

        m_gripper = new Gripper(map);
        m_chassis = new Chassis(map, armM2encoderL.encoder, armM1encoderR.encoder,voltage_sensor);
        m_plane = new plane(map);
        m_climb = new climb(map);
        m_arm = new Arm(map, armM2encoderL, armM1encoderR,voltage_sensor);
        m_pixelDetection=new PixelDetection(map,telemetry);
        new HardwareCache(map);
        oneDriver();
        tune();
    }

    public void tune(){
        m_controller2.assignCommand(m_arm.tuneAngle2(),false,DPAD_UP);
            m_controller2.assignCommand(followPath(TrajectoryFactory._forward),,BUTTON_UP);
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
//
        m_controller.assignCommand(m_plane.shootPlane(), true, BUTTON_DOWN);
        m_controller.assignCommand(m_plane.resetPlane(), true, BUTTON_UP);

        m_controller.assignCommand(m_arm.toggleFF(), false, BUTTON_RIGHT);


        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setHighScore()), false, DPAD_UP);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setScore()), false, DPAD_DOWN);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setIdle()), false, DPAD_LEFT);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setFrontPickup()), false, BUTTON_LEFT);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setPickup()), false, DPAD_RIGHT);

    }

    public FollowPath followPath(Trajectory fromTrajectoryFactory){
        return new FollowPath(fromTrajectoryFactory,fellowPathConfigGen(m_chassis,null));
    }
    public FollowPath.FollowPathConfig fellowPathConfigGen(Chassis m_chassis, @Nullable Rotation2d desiredRotation) {
        BTHolonomicDriveController controller =new BTHolonomicDriveController(new PIDController(0,0,0),new PIDController(0,0,0),new ProfiledPIDController(0,0,0,new TrapezoidProfile.Constraints(0,0)));
        return new FollowPath.FollowPathConfig(
                m_chassis::getPosition,
                controller,
                ()->desiredRotation,
                m_chassis::chassisSpeedDrive,
                m_chassis::resetOdmetry,
                m_chassis
        );
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
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(980)),
                m_chassis.fieldRelativeDrive(0,0,90).andThen(new WaitCommand(500))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(500)),
                m_gripper.openGripper1(),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0.6, 90).andThen(new WaitCommand(440)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive( 0.6,0,90).andThen(new WaitCommand(850)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(700))
                        .andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(0,0,-90).andThen(new WaitCommand(600)),
                m_chassis.stopMotor().andThen(new WaitCommand(300)),
                m_arm.setLowScore().andThen(new WaitCommand(1500)),

                m_gripper.openGripper0(),
                m_chassis.fieldRelativeDrive(0,-0.6,-90).andThen(new WaitCommand(100)),
                m_chassis.stopMotor(),

                m_gripper.closeGripper0(),

                m_arm.setIdle(),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(800)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(-0.6,0,0).andThen(new WaitCommand(800)),
                m_chassis.fieldRelativeDrive(0,-0.6,0).andThen(new WaitCommand(300)),
                m_chassis.stopMotor()
        );
    }
public Command centerCloseRedPath() {
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, 0.6, 0).andThen(new WaitCommand(960)),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(840)),
                m_chassis.fieldRelativeDrive(0,0,-90).andThen(new WaitCommand(500))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(1000)),
                m_gripper.openGripper0(),
                m_gripper.closeGripper0(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,-0.6, -90).andThen(new WaitCommand(500)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive( 0.6,0,-90).andThen(new WaitCommand(380)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(700))
                        .andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(0,0,90).andThen(new WaitCommand(600)),
                m_chassis.stopMotor(),
                m_arm.setScore().andThen(new WaitCommand(1000)),

                m_gripper.openGripper1().andThen(new WaitCommand(400)),
//
                m_gripper.closeGripper1(),

                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0.6,-90).andThen(new WaitCommand(20)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(800)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(-0.6,0,0).andThen(new WaitCommand(800)),
                m_chassis.fieldRelativeDrive(0,0.6,0).andThen(new WaitCommand(300)),
                m_chassis.stopMotor()
        );
    }



    public Command leftCloseBluePath() {
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, -0.6, 0).andThen(new WaitCommand(800)),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(450))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(600)),
                m_gripper.openGripper1(),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(-0.6, 0, 0).andThen(new WaitCommand(50)),
                m_chassis.fieldRelativeDrive(0, 0, -90).andThen(new WaitCommand(450)),
                m_chassis.fieldRelativeDrive(0, 0.6, -90).andThen(new WaitCommand(460))
                        .andThen(m_chassis.stopMotor()),
                        m_chassis.fieldRelativeDrive(-0.6,0,-90).andThen(new WaitCommand(660))
                        .andThen(m_chassis.stopMotor()),

                m_arm.setScore().andThen(new WaitCommand(1000)),
                m_chassis.stopMotor().andThen(new WaitCommand(800)),
                m_gripper.openGripper0().andThen(new WaitCommand(400)),

                m_gripper.closeGripper0(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,-0.6, -90).andThen(new WaitCommand(150)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(800)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(-0.6,0,0).andThen(new WaitCommand(600)),
                m_chassis.fieldRelativeDrive(0,-0.6,0).andThen(new WaitCommand(300)),
                m_chassis.stopMotor()
        );
    }
    public Command rightCloseRedPath() {
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, 0.6, 0).andThen(new WaitCommand(640)),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(450))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(600)),
                m_gripper.openGripper0(),
                m_gripper.closeGripper0(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(-0.6, 0, 0).andThen(new WaitCommand(50)),
                m_chassis.fieldRelativeDrive(0, 0, 90).andThen(new WaitCommand(450)),
                m_chassis.fieldRelativeDrive(0, -0.6, 90).andThen(new WaitCommand(590))
                        .andThen(m_chassis.stopMotor()),
                        m_chassis.fieldRelativeDrive(-0.6,0,90).andThen(new WaitCommand(860))
                        .andThen(m_chassis.stopMotor()),

                m_arm.setScore().andThen(new WaitCommand(1000)),
                m_chassis.stopMotor(),
                m_gripper.openGripper1().andThen(new WaitCommand(400)),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(800)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(-0.6,0,0).andThen(new WaitCommand(800)),
                m_chassis.fieldRelativeDrive(0,0.6,0).andThen(new WaitCommand(300)),
                m_chassis.stopMotor()
        );
    }
    public Command rightCloseBluePath() {
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, -0.6, 0).andThen(new WaitCommand(300)),
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
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0.6,-90).andThen(new WaitCommand(460)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(-0.6,0,-90).andThen(new WaitCommand(400)),
                m_chassis.stopMotor(),
                m_arm.setScore(),
                m_gripper.openGripper0().andThen(new WaitCommand(400)),
                m_chassis.fieldRelativeDrive(0,-0.6,-90).andThen(new WaitCommand(260))
                        .andThen(m_chassis.stopMotor()),
                m_gripper.closeGripper0(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0.6,-90).andThen(new WaitCommand(200)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(800)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(-0.6,0,0).andThen(new WaitCommand(800)),
                m_chassis.fieldRelativeDrive(0,-0.6,0).andThen(new WaitCommand(300)),
                m_chassis.stopMotor()
        );
    }
public Command leftCloseRedPath() {
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, 0.6, 0).andThen(new WaitCommand(320)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(550))
                        .andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(0, 0, 90).andThen(new WaitCommand(400)),
                m_arm.turnOnFF(),
                m_arm.setPickup().andThen(new WaitCommand(600)),
                m_gripper.openGripper0(),
                m_chassis.fieldRelativeDrive(0,-0.6,90).andThen(new WaitCommand(200)),
                m_chassis.stopMotor(),
                m_gripper.closeGripper0(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,-0.6,90).andThen(new WaitCommand(450)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(-0.6,0,90).andThen(new WaitCommand(550)),
                m_chassis.stopMotor(),
                m_arm.setLowScore().andThen(new WaitCommand(1000)),
                m_gripper.openGripper1().andThen(new WaitCommand(400)),
                m_chassis.fieldRelativeDrive(0,0.6,90).andThen(new WaitCommand(400)),
                m_chassis.stopMotor(),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(600)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(-0.6,0,0).andThen(new WaitCommand(800)),
                m_chassis.fieldRelativeDrive(0,0.6,0).andThen(new WaitCommand(800)),
                m_chassis.stopMotor()
        );
    }
//copied
        public Command leftBlueFarPath(){
            return new SequentialCommandGroup(
                    m_gripper.closeGripper0(),
                    m_gripper.closeGripper1(),
                    m_chassis.fieldRelativeDrive(0, 0.6, 0).andThen(new WaitCommand(400)),
                    m_chassis.stopMotor(),
                    m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(590))
                            .andThen(m_chassis.stopMotor()),
                    m_chassis.fieldRelativeDrive(0, 0, 90).andThen(new WaitCommand(400)),
                    m_arm.turnOnFF(),
                    m_arm.setPickup().andThen(new WaitCommand(600)),
                    m_gripper.openGripper0(),
                    m_chassis.fieldRelativeDrive(0, -0.6, 90).andThen(new WaitCommand(200)),
                    m_chassis.stopMotor(),
                    m_gripper.closeGripper0(),
                    m_arm.setIdle(),
                    m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(5000)),
                    m_chassis.stopMotor()




                );

        }

        public Command centerFarBluePath(){
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, 0.6, 0).andThen(new WaitCommand(820)),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(800)),
                m_chassis.fieldRelativeDrive(0,0,-90).andThen(new WaitCommand(500))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(500)),
                m_gripper.openGripper0(),
                m_chassis.fieldRelativeDrive(0, -0.6, -90).andThen(new WaitCommand(200)),
                m_chassis.stopMotor(),
                m_gripper.closeGripper0(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(5000)),
                m_chassis.stopMotor()


        );
        }
//
        public Command rightFarBluePath(){
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, 0.6, 0).andThen(new WaitCommand(700)),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(520))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(600)),
                m_gripper.openGripper0(),
                m_gripper.closeGripper0(),
                m_arm.setIdle()

        );
        }
    public Command rightFarRedPath() {
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, -0.6, 0).andThen(new WaitCommand(270)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(550))
                        .andThen(m_chassis.stopMotor()),
                m_chassis.fieldRelativeDrive(0, 0, -90).andThen(new WaitCommand(400)),
                m_arm.turnOnFF(),
                m_arm.setPickup().andThen(new WaitCommand(600)),
                m_gripper.openGripper1(),
                m_chassis.fieldRelativeDrive(0,0.6,-90).andThen(new WaitCommand(200)),
                m_chassis.stopMotor(),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(5000)),
                m_chassis.stopMotor()


    );
    }
    public Command CenterFarRedPath(){
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, -0.6, 0).andThen(new WaitCommand(670)),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(850)),
                m_chassis.fieldRelativeDrive(0,0,90).andThen(new WaitCommand(500))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(500)),
                m_gripper.openGripper1(),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                m_chassis.fieldRelativeDrive(0,0.6,90).andThen(new WaitCommand(150)),
                m_chassis.stopMotor(),
                m_chassis.fieldRelativeDrive(0,0,0).andThen(new WaitCommand(5000)),
                m_chassis.stopMotor()
        );
    }
    public Command LeftFarRedPath(){
        return new SequentialCommandGroup(
                m_gripper.closeGripper0(),
                m_gripper.closeGripper1(),
                m_chassis.fieldRelativeDrive(0, -0.6, 0).andThen(new WaitCommand(600)),
                m_chassis.fieldRelativeDrive(0.6, 0, 0).andThen(new WaitCommand(450))
                        .andThen(m_chassis.stopMotor()),
                m_arm.turnOnFF(),
                m_arm.setMiddle().andThen(new WaitCommand(600)),
                m_gripper.openGripper1(),
                m_gripper.closeGripper1(),
                m_arm.setIdle()
        );
    }


}