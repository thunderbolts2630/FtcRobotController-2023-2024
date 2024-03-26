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

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
//import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.HardwareCache;
import org.firstinspires.ftc.teamcode.subsystems.pixelDetector.PixelDetection;
import org.firstinspires.ftc.teamcode.subsystems.climb;
import org.firstinspires.ftc.teamcode.subsystems.plane;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

import java.util.List;


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
    public double slowDrive=1;
    @Config
    public static class PidTest{
        public static double desiredX = 0;
        public static double desiredY = 0;
        public static double desiredDegrees = 0;

    }


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
//        m_controller2.assignCommand(m_arm.tuneAngle2(),false,DPAD_UP);
        m_controller2.assignCommand(m_chassis.goToX(1),false,BUTTON_UP).whenInactive(m_chassis.stopMotor());
        m_controller2.assignCommand(m_chassis.goToY(1),false,BUTTON_DOWN).whenInactive(m_chassis.stopMotor());
        m_controller2.assignCommand(m_chassis.goToDegrees(90),false,BUTTON_LEFT).whenInactive(m_chassis.stopMotor());
        m_controller2.assignCommand(m_arm.turnOnFF(),false,BUTTON_RIGHT).whenInactive(m_chassis.stopMotor());
        m_controller2.assignCommand(m_arm.goTo(Constants.ArmConstants.Positions.MID_PICKUP_FRONT_CLOSE),false,DPAD_UP);
        m_controller2.assignCommand(m_arm.goTo(Constants.ArmConstants.Positions.IDLE),false,DPAD_DOWN);
    }


    public double squareInput(double input){
        return Math.signum(input)*Math.pow(input,2);
    }
    //bind commands to trigger
    public void oneDriver(){

        m_controller.assignCommand(m_chassis.fieldRelativeDrive(
                        () -> squareInput(-m_controller.left_y.getAsDouble()),
                        () -> squareInput(m_controller.left_x.getAsDouble()),
                        () -> squareInput(m_controller.right_trigger.getAsDouble() - m_controller.left_trigger.getAsDouble())),
                true, LEFT_X, LEFT_Y, LEFT_TRIGGER, RIGHT_TRIGGER).whenInactive(m_chassis.stopMotor());
        m_controller.assignCommand(m_climb.climb_manual(() -> -m_controller.right_x.getAsDouble()), true, RIGHT_X).whenInactive(m_climb.climb_manual(() -> 0));


        m_controller.assignCommand(m_gripper.toggleGripper1(), false, BUMPER_RIGHT);
        m_controller.assignCommand(m_gripper.toggleGripper0(), false, BUMPER_LEFT);
//
        m_controller.assignCommand(m_plane.shootPlane(), false, BUTTON_DOWN);
        m_controller.assignCommand(m_plane.resetPlane(), false, BUTTON_UP);

        m_controller.assignCommand(m_arm.toggleFF(), false, BUTTON_RIGHT);

        m_controller.assignCommand(m_arm.openDoorway(),false,BUTTON_DOWN);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setHighScore()).alongWith(m_chassis.setDriveSpeed(0.5)), false, DPAD_UP);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setScore()).alongWith(m_chassis.setDriveSpeed(0.5)), false, DPAD_DOWN);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setIdle()).alongWith(m_chassis.setDriveSpeed(1)), false, DPAD_LEFT);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setFrontPickup()).alongWith(m_chassis.setDriveSpeed(0.5)), false, DPAD_RIGHT);
        m_controller.assignCommand(m_gripper.closeBoth().andThen(m_arm.setLowScore()).alongWith(m_chassis.setDriveSpeed(0.5)), false, BUTTON_LEFT);

    }

    public Command withTimeout(Command command,int msTimeout){
        ElapsedTime time=new ElapsedTime();

        return  new FunctionalCommand(()->{command.initialize(); time.reset();},command::execute,command::end,()->command.isFinished()||time.milliseconds()>msTimeout);
    }
    public Command farBlueCenterAutoBT() {
        return new SequentialCommandGroup(
                m_gripper.closeGripper1(),
                m_gripper.closeGripper0(),
                m_chassis.goToX(0.325),
                m_chassis.stopMotor(),
                m_arm.turnOnFF(),
                new ParallelRaceGroup(m_arm.setFrontPickup(),new WaitCommand(2000)),
                m_chassis.goToX(-0.25),
                m_gripper.openGripper1(),
                m_arm.goTo(Constants.ArmConstants.Positions.MID_PICKUP_FRONT),
                m_gripper.closeGripper1(),
                m_arm.setIdle()

        );
    }
 public Command closeBlueCenterAutoBT() {
        return new SequentialCommandGroup(
                new WaitCommand(700),
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_gripper.closeGripper1(),
                m_gripper.closeGripper0(),
                m_chassis.goToX(0.2),
                withTimeout(m_chassis.goToY(0.28),800),
                withTimeout(m_chassis.goToDegrees(0),800),
                m_chassis.stopMotor(),
                m_arm.turnOnFF(),
                withTimeout(m_arm.goTo(Constants.ArmConstants.Positions.MID_PICKUP_FRONT),2000),
                new WaitCommand(1000),
                m_gripper.openGripper0(),
                m_gripper.closeGripper0(),
                m_arm.setIdle(),
                m_chassis.goToX(-0.1),
                withTimeout(m_chassis.goToY(-0.86),4000),
                m_chassis.stopMotor(),
                withTimeout(m_chassis.goToDegrees(0),1000),
                m_chassis.goToX(0.17),
                withTimeout(m_chassis.goToDegrees(-90),1500),
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToX(0.62),
                withTimeout(m_arm.goTo(Constants.ArmConstants.Positions.LOWERLOWSCORE),2000),
                new WaitCommand(600),
                m_gripper.openGripper1(),
                m_chassis.goToX(-0.1),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                m_chassis.goToDegrees(90),
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToX(-0.8),
                m_chassis.stopMotor()

        );
    }


    public Command closeBlueRightAutoBT() {
        return new SequentialCommandGroup(
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_gripper.closeGripper1(),
                m_gripper.closeGripper0(),
                m_chassis.goToY(-0.52),
                m_chassis.stopMotor(),
                m_chassis.goToDegrees(0),
                m_chassis.stopMotor(),
                m_chassis.goToX(0.6),
                m_chassis.stopMotor(),
                withTimeout(m_chassis.goToDegrees(92),3000),
                m_chassis.stopMotor(),
                m_arm.turnOnFF(),
                withTimeout(m_arm.goTo(Constants.ArmConstants.Positions.MID_PICKUP_FRONT),2000),
                new WaitCommand(400),
                m_gripper.openGripper0().andThen(new WaitCommand(800)),
                m_gripper.closeGripper0(),
                withTimeout(m_arm.setIdle(),1500),
                withTimeout(m_chassis.goToDegrees(-90),3000),
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToX(0.47),
                m_chassis.goToY(0.03),
                withTimeout(m_chassis.goToDegrees(0),2500),
                m_chassis.stopMotor(),
                withTimeout(m_arm.setLowScore(),1200),
                new WaitCommand(1000),
                m_gripper.openGripper1().andThen(new WaitCommand(800)),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                withTimeout(m_chassis.goToDegrees(90),3000),
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToX(-0.7),
                m_chassis.stopMotor()

        );
    }

    public Command closeBlueLeftAutoBT() {
        return new SequentialCommandGroup(
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_gripper.closeGripper1(),
                m_gripper.closeGripper0(),
                m_chassis.goToY(-0.52),
                m_chassis.stopMotor(),
                m_chassis.goToDegrees(0),
                m_chassis.stopMotor(),
                m_chassis.goToX(0.6),
                m_chassis.stopMotor(),
                withTimeout(m_chassis.goToDegrees(92),3000),
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToX(-0.2),
                m_chassis.stopMotor(),
                m_arm.turnOnFF(),
                withTimeout(m_arm.goTo(Constants.ArmConstants.Positions.MIDDLEPLUSPLUS),2000),
                new WaitCommand(700),
                m_gripper.openGripper0().andThen(new WaitCommand(800)),
                m_gripper.closeGripper0(),
                withTimeout(m_arm.setIdle(),1500),
                withTimeout(m_chassis.goToDegrees(180),2500),
                m_chassis.stopMotor(),
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToY(-0.28),
                m_chassis.goToX(0.05),
                m_chassis.stopMotor(),
                withTimeout(m_arm.setLowScore(),2000),
                new WaitCommand(1000),
                m_gripper.openGripper1().andThen(new WaitCommand(800)),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                withTimeout(m_chassis.goToDegrees(90),3000),
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToX(-0.5),
                m_chassis.stopMotor()



        );
    }
    //adapted from blue left
    public Command closeRedRightAutoBT() {
        return new SequentialCommandGroup(
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_gripper.closeGripper1(),
                m_gripper.closeGripper0(),
                m_chassis.goToX(0.33),
                m_chassis.goToY(0.29),//flipped
                m_chassis.stopMotor(),
                m_chassis.goToDegrees(0),
                m_chassis.stopMotor(),
                m_chassis.goToX(0.6),
                m_chassis.stopMotor(),
                withTimeout(m_chassis.goToDegrees(-92),3000),//flipped
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToX(-0.2),
                m_chassis.stopMotor(),
                m_arm.turnOnFF(),
                withTimeout(m_arm.goTo(Constants.ArmConstants.Positions.MIDDLEPLUSPLUS),2000),
                new WaitCommand(700),
                m_gripper.openGripper0().andThen(new WaitCommand(800)),
                m_gripper.closeGripper0(),
                withTimeout(m_arm.setIdle(),1500),
                withTimeout(m_chassis.goToDegrees(-180),2500),//flipped
                m_chassis.stopMotor(),
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToY(0.08),//
                withTimeout(m_chassis.goToX(0.15),1200),
                m_chassis.stopMotor(),
                withTimeout(m_arm.goTo(Constants.ArmConstants.Positions.LOWERLOWSCORE),2000),
                new WaitCommand(1000),
                m_gripper.openGripper1().andThen(new WaitCommand(800)),
                m_chassis.goToX(-0.07),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                withTimeout(m_chassis.goToDegrees(-90),3000),//flipped
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToX(-0.5),
                m_chassis.stopMotor()

        );
    }

    public Command closeRedLeftAutoBT() {
        return new SequentialCommandGroup(
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_gripper.closeGripper1(),
                m_gripper.closeGripper0(),
                m_chassis.goToX(0.29),//added
                m_chassis.goToY(0.32),//flipped
                m_chassis.stopMotor(),
                m_chassis.goToDegrees(0),
                m_chassis.stopMotor(),
                m_chassis.goToX(0.6),
                m_chassis.stopMotor(),
                withTimeout(m_chassis.goToDegrees(-92),3000),//flipped
                m_chassis.stopMotor(),
                m_arm.turnOnFF(),
                withTimeout(m_arm.goTo(Constants.ArmConstants.Positions.MID_PICKUP_FRONT),2000),
                new WaitCommand(400),
                m_gripper.openGripper0().andThen(new WaitCommand(800)),
                m_gripper.closeGripper0(),
                withTimeout(m_arm.setIdle(),1500),
                withTimeout(m_chassis.goToDegrees(-90),3000),//flipped
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToX(0.57),
                m_chassis.goToY(-0.03),//flipped
                withTimeout(m_chassis.goToDegrees(0),2500),
                m_chassis.stopMotor(),
                withTimeout(m_arm.goTo(Constants.ArmConstants.Positions.LOWERLOWSCORE),1200),
                new WaitCommand(1000),
                m_gripper.openGripper1().andThen(new WaitCommand(800)),
                m_chassis.goToX(-0.07),
                m_gripper.closeGripper1(),
                m_arm.setIdle(),
                withTimeout(m_chassis.goToDegrees(-90),3000),//flipped
                new InstantCommand(()->m_chassis.resetOdmetry(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
                m_chassis.goToX(-0.7),
                m_chassis.stopMotor()
        );
    }

    /*
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
*/
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