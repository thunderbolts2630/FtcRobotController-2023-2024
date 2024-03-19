package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.BT.hardware.BTLynxDCMotorController;
import org.firstinspires.ftc.teamcode.utils.BT.hardware.BTLynxServoController;

import java.util.Collection;
import java.util.List;

@TeleOp(name = "test bench")
public class testbench extends LinearOpMode {
    int testCounts=1000;
    int i=0;
    volatile double ser0;
    volatile double t;
    volatile double v;
    volatile double ser1;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime elapsedTime= new ElapsedTime();
        Servo grip1= hardwareMap.servo.get("gripper1");
        Servo grip0= hardwareMap.servo.get("gripper0");
        Servo armserv= hardwareMap.servo.get("armServo");
        DcMotor motor= hardwareMap.get(DcMotor.class,"motor_FR");
        DcMotor motor1= hardwareMap.dcMotor.get("motor_BR");
        DcMotor motor2= hardwareMap.dcMotor.get("motor_BL");
        DcMotor motor3= hardwareMap.dcMotor.get("motor_BR");
        DcMotor motor4 =  hardwareMap.dcMotor.get( "climb_motor");
        DcMotor motor5 = hardwareMap.dcMotor.get( "ArmM1encoderR");//3
        DcMotor motor6 = hardwareMap.dcMotor.get( "ArmM2encoderC");//0
        DcMotor motor7 = hardwareMap.dcMotor.get( "encoderLeft");//0
        List<LynxModule> hubs= hardwareMap.getAll(LynxModule.class);
        hubs.forEach(lynxModule -> lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        AnalogInput potentiometer1 = hardwareMap.get(AnalogInput.class, "pt1");//port 3
        AnalogInput potentiometer2 = hardwareMap.get(AnalogInput.class, "pt2");//port 1
        LynxModule usedModuleControl = hardwareMap.get(LynxModule.class,"Control Hub");//Expansion Hub 2 is the other name
        LynxModule usedModuleControlExpansion = hardwareMap.get(LynxModule.class,"Expansion Hub 2");//Expansion Hub 2 is the other name

        BTLynxDCMotorController btLynxDCMotorController;
        BTLynxDCMotorController btlExpa;
        BTLynxServoController btServoController;
        try {
            btLynxDCMotorController= new BTLynxDCMotorController(hardwareMap.appContext, usedModuleControl);
            btlExpa= new BTLynxDCMotorController(hardwareMap.appContext,usedModuleControlExpansion);
            btServoController=new BTLynxServoController(hardwareMap.appContext, usedModuleControl);
        } catch (RobotCoreException e) {
            telemetry.addLine(e.getMessage());
            telemetry.update();
            throw new RuntimeException(e);
        }
        if (btLynxDCMotorController==null){

            telemetry.addLine("failed");
            telemetry.update();
            while (opModeIsActive()){}
        }
        DcMotorImpl bettermotor=new DcMotorImpl(btLynxDCMotorController,motor.getPortNumber());
        DcMotorImpl bettermotor1=new DcMotorImpl(btLynxDCMotorController,motor1.getPortNumber());
        DcMotorImpl bettermotor3=new DcMotorImpl(btLynxDCMotorController,motor3.getPortNumber());
        DcMotorImpl bettermotor2=new DcMotorImpl(btLynxDCMotorController,motor2.getPortNumber());
        DcMotorImpl bettermotor5=new DcMotorImpl(btlExpa,motor5.getPortNumber());
        Servo bettergrip0 = new ServoImpl(btServoController,grip0.getPortNumber());
        Servo bettergrip1 = new ServoImpl(btServoController,grip1.getPortNumber());
        Servo betterArmServo = new ServoImpl(btServoController,armserv.getPortNumber());
        Runnable runnable=()->{
            bettergrip1.setPosition(0.3+0.001*Math.sin(elapsedTime.milliseconds()));
            bettergrip0.setPosition(0.3+0.001*Math.sin(elapsedTime.milliseconds()));
            betterArmServo.setPosition(0.3+0.01*Math.sin(elapsedTime.milliseconds()));
            ser1=grip1.getPosition();
            ser0=bettermotor.getCurrentPosition();
            v=potentiometer1.getVoltage();
            v=potentiometer2.getVoltage();
            ser1=grip1.getPosition();
            ser0=bettermotor.getCurrentPosition();
            v=bettermotor1.getCurrentPosition();
            v=bettermotor.getCurrentPosition();
            v=bettermotor5.getCurrentPosition();
            v=motor7.getCurrentPosition();
            v=motor4.getCurrentPosition();
            t=bettermotor2.getCurrentPosition();
            bettermotor2.setPower(0.1+0.05*Math.sin(elapsedTime.seconds()));
            bettermotor.setPower(0.1+0.05*Math.sin(elapsedTime.seconds()));
            bettermotor3.setPower(0.1+0.05*Math.sin(elapsedTime.seconds()));
            bettermotor1.setPower(0.1+0.05*Math.sin(elapsedTime.seconds()));
            motor4.setPower(0.01+0.009*Math.sin(elapsedTime.seconds()));
            bettermotor5.setPower(0.01+0.009*Math.sin(elapsedTime.seconds()));
//            bettermotor6.setPower(0.01+0.009*Math.sin(elapsedTime.seconds()));
            hubs.forEach(module -> module.clearBulkCache());
        };

        waitForStart();
        elapsedTime.reset();
        while (opModeIsActive()&& i<testCounts ){
            i++;
            runnable.run();
        }
        telemetry.addData("loop time with auto",elapsedTime.seconds()/testCounts);
        hubs.forEach(lynxModule -> lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.OFF));

        telemetry.update();
        while (opModeIsActive()){
            v=0;
        }


    }

}
