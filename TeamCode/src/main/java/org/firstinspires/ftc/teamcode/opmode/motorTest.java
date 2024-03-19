package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

@TeleOp(name = "motor test")
public class motorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor= hardwareMap.dcMotor.get("motor_FR");
        DcMotor motor1= hardwareMap.dcMotor.get("motor_FL");
        DcMotor motor2= hardwareMap.dcMotor.get("motor_BL");
        DcMotor motor3= hardwareMap.dcMotor.get("motor_BR");
        waitForStart();
        while (opModeIsActive()){
            motor1.setPower(0.12);
            motor3.setPower(0.12);
            motor2.setPower(0.12);
            motor.setPower(0.12);
        }
    }
}
