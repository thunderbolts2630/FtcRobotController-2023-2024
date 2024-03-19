package org.firstinspires.ftc.teamcode.utils.BT.hardware;

import static com.qualcomm.robotcore.hardware.configuration.LynxConstants.NUMBER_OF_SERVO_CHANNELS;

import android.content.Context;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.Range;

public class BTLynxServoController extends LynxServoController {
    boolean[] servoEnabled;

    public BTLynxServoController(Context context, LynxModule module) throws RobotCoreException, InterruptedException {
        super(context, module);
        servoEnabled =new boolean[NUMBER_OF_SERVO_CHANNELS];
        for (int i = 0; i < servoEnabled.length; i++) {
            servoEnabled[i]=false;
        }
    }

    public synchronized void setServoPosition(int servo, double position) {
        servo -= -apiServoFirst;
        position = Range.clip(position, 0, 1);
        if (lastKnownCommandedPosition[servo].updateValue(position)) {
            double pwm = Range.scale(position, apiPositionFirst, apiPositionLast, pwmRanges[servo].usPulseLower, pwmRanges[servo].usPulseUpper);
            pwm = Range.clip(pwm, LynxSetServoPulseWidthCommand.apiPulseWidthFirst, LynxSetServoPulseWidthCommand.apiPulseWidthLast);
            LynxSetServoPulseWidthCommand command = new LynxSetServoPulseWidthCommand(this.getModule(), servo, (int) pwm);
            try {
                try {
                    command.acquireNetworkLock();
                    command.getModule().sendCommand(command);
                    command.pretendFinish();
                } catch (InterruptedException | LynxUnsupportedCommandException e) {
                    throw new RuntimeException(e);
                } finally {
                    command.releaseNetworkLock();

                }
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            if(!servoEnabled[servo])
            {
                // Auto-enable after setting position to match historical behavior (and because it's handy)
                super.setServoPwmEnable(servo);
            }
        }
    }
}
