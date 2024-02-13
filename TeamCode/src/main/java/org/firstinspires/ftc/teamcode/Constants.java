package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.ChassisFeedForward.*;
import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.PIDConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.utils.geometry.BTTranslation2d;

public class Constants {
    public static final double l1 = 0.378;// com distant from axis first arm METERS
    public static final double l2 = 0.355;// com distant from axis second arm METERS
    public static final double first_arm_weight = 0.200; // first arm weight KG
    public static final double second_arm_weight = 0.200; // second arm weight with gripper KG
    public static final double g = 9.806;
    public static final double hex_stall_current = 9.801;
    public static final double resistance = 12 / hex_stall_current; //volt

    public static final double first_gear_ratio = 3;
    public static final double second_gear_ratio = 1.33333;
    public static final double hex_stall_torque = 0.173; //N * meter

    public static final double neo_Kt = hex_stall_torque / hex_stall_current;

    public static class ArmConstants {

        public static final double motorMaxVolt = 12;
        public static final double a1Max = 90;
        public static final double vMax1 = 1.456;
        public static final double a2Max = 0;
        public static final double vMin2 = 1.632;
        public static final double arm1Min = 0;
        public static final double vMin1 = 0.455;
        public static final double arm2Min = -90;
        public static final double vMax2 = 2.189;
        public enum Positions{
            DROP(1.848,1.797,0.65,0,0),
            IDLE(0,0,0.6,0,0),
            TAKE(2.124,2.549,0.6,0,0);
            double v1,v2,servo;//this is checked 2/13/24 (m/d/y)
            double ff1,ff2;//this is not yet checked 2/13/24 (m/d/y)

            Positions(double v1, double v2, double servo, double ff1, double ff2) {
                this.v1 = v1;
                this.v2 = v2;
                this.servo = servo;
                this.ff1 = ff1;
                this.ff2 = ff2;
            }


        }

        @Config
        public static class ArmPID{
            public static double a1KP = 0.0;
            public static double a2KP = 0.0;
            public static double a1KI = 0.00;
            public static double a2KI = 0.00;
            public static double a1KD = 0;
            public static double a2KD = 0;
            public static double ffConv=12;
        }
        @Config
        public static class calib{
            public static double armServo=0.6;
            public static double arm1=0.0;
            public static double arm2=0.0;

        }


        public static final SimpleMotorFeedforward c_arm1FF = new SimpleMotorFeedforward(0,1,0);
        public static  final SimpleMotorFeedforward c_arm2FF = new SimpleMotorFeedforward(0.15,1,0);

    }

    public static class ChassisConstants {
        public static final TrapezoidProfile.Constraints TrapezoidConstraints = new TrapezoidProfile.Constraints(1.63, 1.47);
        public static final double odometryWheelRadius = 0.0176; //meters
        public static final int tickPerRevolution = 8192;
        public static final double TimeToAprilTagCheck = 1;
        public static final double TRACKWIDTH = 0.29; //wheelbase, change value
        public static final double WHEEL_OFFSET = 1;
        public static final double TICKS_TO_CM = 38.862;
        public static final double RobotMaxVelFront = 1.63; // m/s
        public static final double RobotMaxVelSide = 0.89; // m/s
        public static final double RobotMaxAccFront = 1.47; // m/s^2
        public static final BTTranslation2d FRW = new BTTranslation2d(0.145,0.137);
        public static final BTTranslation2d BRW = new BTTranslation2d(0.145,-0.137 );
        public static final BTTranslation2d FLW = new BTTranslation2d(-0.145,0.137);
        public static final BTTranslation2d BLW = new BTTranslation2d(-0.145,-0.137);
        public static SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(ffks,ffkv,ffka);
        @Config
        public static class ChassisFeedForward{
            public static double ffks = 0.12;
            public static double ffkv = 0;
            public static double ffka = 0;
        }

        public static final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(FLW,FRW,BLW,BRW);
        public static final double  robotThetaVelocityMax= 180; //degree per sec
        public static final double  robotThetaAccMax= 180; //degree per sec^2

        public static final PIDController PIDy = new PIDController(kpY,kiY,kdY);
        public static final ProfiledPIDController PIDt = new ProfiledPIDController(kpT,kiT,kdT,
                new TrapezoidProfile.Constraints(RobotMaxVelFront,RobotMaxAccFront));
        public static final PIDController PIDx = new PIDController(kpX,kiX,kdX);
        @Config
        public static class PIDConstants {

            public static double kpX = 0;
            public static double kiX = 0;
            public static double kdX = 0;
            public static double kpY = 0;
            public static double kiY = 0;
            public static double kdY = 0;
            public static double kpT = 0;
            public static double kiT = 0;
            public static double kdT = 0;
            public static double kp = 0;
            public static double ki = 0;
            public static double kd = 0;
            public static double kff = 0;
        }
    }





    final public static double cameraAngle = 0;
    public static class  Climb{
        public static final double climb_max_speed = 0;// todo: this is not calibrated
        public static final double climb_max_accel = 0;// todo: this is not calibrated
        public static final double kp = 0;// todo: this is not calibrated
        public static final double ki = 0;// todo: this is not calibrated
        public static final double kd = 0;// todo: this is not calibrated
        public static final double kf = 0;// todo: this is not calibrated
        public static final int max_ticks = 0;// todo: this is not calibrated

//    public enum ArmStates {
//        base(Arm.armBasePosition,false,ArmPlacingStates.base), // this doesn't need any boolean has a command on his own
//
//        public Translation2d desiredPoint;
//        //false is positive x true is negative
//        public boolean direction;
//        public ArmPlacingStates placingHeight;
//
//        private ArmStates(Translation2d point) {
//            this.desiredPoint = point;
//
//        }
//    }
//
}
}
