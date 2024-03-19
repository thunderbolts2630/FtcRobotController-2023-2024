package org.firstinspires.ftc.teamcode.utils.BT.hardware;


import android.content.Context;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetVelocityCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Map;

public class BTLynxDCMotorController extends LynxDcMotorController {

    int bufferLen;
    ArrayList<LynxCommand> commandsQueue;
    boolean[] motorsEnabled;
    public BTLynxDCMotorController(Context context, LynxModule module) throws RobotCoreException, InterruptedException {
        super(context, module);
        motorsEnabled=new boolean[LynxConstants.NUMBER_OF_MOTORS];
        for (int i = 0; i < 4; i++) {
            motorsEnabled[i]=false;
        }
        commandsQueue=new ArrayList<>(LynxConstants.NUMBER_OF_MOTORS);
        bufferLen=0;
    }


    public synchronized void sendQueuedCommands(){

        ByteBuffer buffer=ByteBuffer.allocate(bufferLen);
        for (int i = 0; i < commandsQueue.size(); i++) {
            buffer.put(commandsQueue.get(i).getSerialization().toByteArray());
        }


    }
    @Override
    public synchronized void setMotorPower(int motor, double apiMotorPower) {
        validateMotor(motor);
        int motorZ = motor - apiMotorFirst;//
        double power = Range.clip(apiMotorPower, apiPowerFirst, apiPowerLast);
        int iPower = 0;
        power = Range.scale(power, apiPowerFirst, apiPowerLast, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast);
        iPower = (int) power;
        LynxCommand command = new LynxSetMotorConstantPowerCommand(this.getModule(), motorZ, iPower);
        // Serialize this guy and remember it
        try {
            if (DEBUG) {
                RobotLog.vv(TAG, "setMotorPower: mod=%d motor=%d iPower=%d", getModuleAddress(), motorZ, iPower);
            }
            command.acquireNetworkLock();
            try {

                command.getModule().sendCommand(command);//by sending the command directly to the lynx module we bypass waits for response
                command.pretendFinish();
            } catch (LynxUnsupportedCommandException e) {
                throw new RuntimeException(e);
            } finally {
                command.releaseNetworkLock();
            }

            if(!motorsEnabled[motor]) {
                super.setMotorEnable(motorZ);
                motorsEnabled[motorZ]=true;
            }
        } catch (InterruptedException | RuntimeException e) {
            handleException(e);
        }
    }


    private void validateMotor(int motor) {
        if (motor < apiMotorFirst || motor > apiMotorLast) {
            throw new IllegalArgumentException(String.format("motor %d is invalid; valid motors are %d..%d", motor, apiMotorFirst, apiMotorLast));
        }
    }

    /*

     */
}
