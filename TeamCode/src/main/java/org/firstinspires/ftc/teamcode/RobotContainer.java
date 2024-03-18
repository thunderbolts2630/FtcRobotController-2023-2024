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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.Path.FollowPath;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
//import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.pixelDetector.PixelDetection;
import org.firstinspires.ftc.teamcode.subsystems.climb;
import org.firstinspires.ftc.teamcode.subsystems.plane;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;
import org.firstinspires.ftc.teamcode.utils.BT.BTHolonomicDriveController;
import org.firstinspires.ftc.teamcode.utils.BT.hardware.BTLynxDCMotorController;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;

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
    Gamepad gamepad1;
    Gamepad gamepad2;
    VoltageSensor voltage_sensor;
    public PixelDetection m_pixelDetection;
    public static double armAccAdjustment = 0;
    DcMotorImpl motor_FR,motor_BR,motor_BL,motor_FL, motor_armM2encoderL, motor_armM1encoderR,motor_climb;



    public RobotContainer(HardwareMap map, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        initializeDevices(map);
        //enable bulk read

        voltage_sensor =  map.voltageSensor.iterator().next();

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        m_controller = new BTController(gamepad1);
        m_controller2 = new BTController(gamepad2);

        m_gripper = new Gripper(map);
        m_chassis = new Chassis(map, motor_armM2encoderL.encoder, motor_armM1encoderR.encoder,voltage_sensor);
        m_plane = new plane(map);
        m_climb = new climb(map);
        m_arm = new Arm(map, motor_armM2encoderL, motor_armM1encoderR,voltage_sensor);
        m_pixelDetection=new PixelDetection(map,telemetry);

        oneDriver();
        tune();
    }
    public void initializeDevices(HardwareMap hardwareMap){
        DcMotor motorFR= hardwareMap.dcMotor.get( "motor_FR");
        DcMotor motorFL= hardwareMap.dcMotor.get("motor_FL");
        DcMotor motorBL= hardwareMap.dcMotor.get("motor_BL");
        DcMotor motorBR= hardwareMap.dcMotor.get("motor_BR");
        DcMotor motorArmM1encoderR= hardwareMap.dcMotor.get("ArmM1encoderR");
        DcMotor motorArmM2encoderC= hardwareMap.dcMotor.get("ArmM2encoderC");
        DcMotor motorClimb= hardwareMap.dcMotor.get("climb_motor");
        List<LynxModule> hubs= hardwareMap.getAll(LynxModule.class);
        hubs.forEach(lynxModule -> lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));
        BTLynxDCMotorController btLynxDCControlHub;
        BTLynxDCMotorController btLynxDCExpansionHub;
        LynxModule ControlHub = hardwareMap.get(LynxModule.class,"Control Hub");
        LynxModule ExpansionHub = hardwareMap.get(LynxModule.class,"Expansion Hub 2");
        try {
            btLynxDCControlHub= new BTLynxDCMotorController(hardwareMap.appContext, ControlHub);
            btLynxDCExpansionHub= new BTLynxDCMotorController(hardwareMap.appContext, ExpansionHub);
        } catch (RobotCoreException | InterruptedException e) {
            throw new RuntimeException(e);
        }
        motor_FR=new DcMotorImpl(btLynxDCControlHub,motorFR.getPortNumber());
        motor_FL=new DcMotorImpl(btLynxDCControlHub,motorFL.getPortNumber());
        motor_BR=new DcMotorImpl(btLynxDCControlHub,motorBR.getPortNumber());
        motor_BL=new DcMotorImpl(btLynxDCControlHub,motorBL.getPortNumber());
        motor_armM1encoderR =new DcMotorImpl(btLynxDCExpansionHub, motorArmM1encoderR.getPortNumber());
        motor_armM1encoderR =new DcMotorImpl(btLynxDCExpansionHub, motorArmM2encoderC.getPortNumber());
        motor_climb= new DcMotorImpl(btLynxDCExpansionHub,motorClimb.getPortNumber());
    }
    public void tune(){
        m_controller2.assignCommand(m_arm.tuneAngle2(),false,DPAD_UP);
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

    public FollowPath.FellowPathConfig fellowPathConfigGen(Chassis m_chassis, @Nullable Rotation2d desiredRotation) {
        BTHolonomicDriveController controller =new BTHolonomicDriveController(new PIDController(0,0,0),new PIDController(0,0,0),new ProfiledPIDController(0,0,0,new TrapezoidProfile.Constraints(0,0)));
        return new FollowPath.FellowPathConfig(
                m_chassis::getPosition,
                controller,
                ()->desiredRotation,
                m_chassis::chassisSpeedDrive,
                m_chassis::resetOdmetry,
                m_chassis
        );
    }



}