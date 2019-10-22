package org.firstinspires.ftc.teamcode.Skystone2019.Config;

import com.qualcomm.hardware.lynx.LynxUsbDeviceDelegate;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

import java.util.HashSet;
import java.util.Set;

public class ConfigFactory {

    private HashSet<String> foundDevices = new HashSet<>();

    public ConfigFactory()
    {
        HardwareMap hardwareMap = HardwareMecanumBase.HardwareMap;

        for (HardwareDevice device : hardwareMap)
        {
            if (device instanceof LynxUsbDeviceDelegate)
            {
                foundDevices.add(((LynxUsbDeviceDelegate) device).getSerialNumber().toString());
            }
        }
    }

    public IConfiguration Get() {
        if (foundDevices.contains("DQ2M7F99"))
        {
            return new RobotConfigurationC();
        }
        else
        {
            return new RobotConfigurationDefault();
        }
    }
}
