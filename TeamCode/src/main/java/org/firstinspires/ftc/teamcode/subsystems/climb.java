package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;
import org.firstinspires.ftc.teamcode.utils.RunCommand;

import static org.firstinspires.ftc.teamcode.Constants.Climb.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class climb  implements Subsystem{

//    TrapezoidProfile profile =,    // Creates a new TrapezoidProfile | Profile will have a max vel of 5 meters per second | Profile will have a max acceleration of 10 meters per second squared | Profile will end stationary at 5 meters | Profile will start stationary at zero position
//            new TrapezoidProfile.State(5, 0),
//            new TrapezoidProfile.State(0, 0));

    PIDFController m_pidfController;
    TrapezoidProfile.Constraints m_sysConstraints;
    TrapezoidProfile m_profile;
    ElapsedTime m_elapsedTime;

    private HardwareMap map;
    private DcMotorImplEx climb_motor;
    private boolean isUp;


    public climb(HardwareMap map, DcMotorImplEx motorClimb) {
        this.map = map;
        m_elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        m_pidfController = new PIDFController(kp, ki, kd, kf);
        climb_motor = motorClimb;
        m_sysConstraints = new TrapezoidProfile.Constraints(climb_max_speed, climb_max_accel);
        isUp = false;
        climb_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        climb_motor.setZeroPowerBehavior(BRAKE);
        register();
    }


    @Override
    public void periodic() {
//        FtcDashboard.getInstance().getTelemetry().addData("climb encoder", climb_motor.getCurrentPosition());
    }

    public void stopMotors() {
        climb_motor.setPower(0);
    }
    public BTCommand climb_manual(DoubleSupplier speed){
        return new RunCommand(()->{
            double value=speed.getAsDouble();//initial value;
            if((speed.getAsDouble()>0 &&climb_motor.getCurrentPosition()>max_ticks) ||(speed.getAsDouble()<0 &&climb_motor.getCurrentPosition()<min_ticks)){
                value=0;
            }else {
                value=speed.getAsDouble();
            }
            climb_motor.setPower(value);
            FtcDashboard.getInstance().getTelemetry().addData("climb asafsd",value);

        });
    }




}






