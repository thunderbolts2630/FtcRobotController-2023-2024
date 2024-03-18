package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class HardwareCache implements Subsystem {
    List<LynxModule> hubs;

    public HardwareCache(HardwareMap map) {
        hubs=map.getAll(LynxModule.class);
        hubs.forEach(lynxModule -> lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        register();
    }

    @Override
    public void periodic() {
        hubs.forEach(LynxModule::clearBulkCache);
    }
}
