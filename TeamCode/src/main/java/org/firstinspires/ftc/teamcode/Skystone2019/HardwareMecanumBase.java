/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 *  Hub 2
 * Motor channel:        "left_front"        motor 0
 * Motor channel:        "right_front"       motor 1
 * Motor channel:        "left_back"         motor 2
 * Motor channel:        "right_back"        motor 3
 *
 *  Hub 1
 * Motor channel:        "lift"              motor1
 *
 *
 * Motors: NeveRest 20 Gearmotor (am-3637)
 *   Theoretical Performance Specifications:
 *   Gearbox Reduction: 19.2:1
 *   Voltage: 12 Volt DC
 *   No Load Free Speed, at gearbox output shaft: 340 RPM
 *   Force Needed to Back-Drive: 6.4 oz-in
 *   Gearbox Output Power: 14 W
 *   Stall Torque: 175 oz-in
 *   Stall Current: 11.5 A
 *   Output counts per revolution of Output Shaft (cpr): 537.6
 *   Output pulse per revolution of encoder shaft (ppr): 134.4

 */
public class HardwareMecanumBase {

    public static HardwareMap HardwareMap = null;

    public DcMotor left_front_drive = null;
    public DcMotor right_front_drive = null;
    public DcMotor left_back_drive = null;
    public DcMotor right_back_drive = null;
    public DcMotor Block_Lifter = null;
    public DcMotor Block_Grabber = null;
    public DcMotor ElevatorClaw = null;
    public DcMotor ElevatorLifter = null;
    public Rev2mDistanceSensor Distance_Sensor = null;
    public ModernRoboticsI2cGyro ExternalGyro = null;
    public BNO055IMU IMU = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        HardwareMecanumBase.HardwareMap = ahwMap;

        // Define and Initialize Motors
        try {
            left_front_drive = HardwareMecanumBase.HardwareMap.get(DcMotor.class, "left_front");
        }
        catch (IllegalArgumentException ex) {}
        try {
            right_front_drive = HardwareMecanumBase.HardwareMap.get(DcMotor.class, "right_front");
        }
        catch (IllegalArgumentException ex) {}
        try {
            left_back_drive = HardwareMecanumBase.HardwareMap.get(DcMotor.class, "left_back");
        }
        catch (IllegalArgumentException ex) {}
        try {
            right_back_drive = HardwareMecanumBase.HardwareMap.get(DcMotor.class, "right_back");
        }
        catch (IllegalArgumentException ex) {}
        try {
            Block_Lifter = HardwareMecanumBase.HardwareMap.get(DcMotor.class, "Block_Lifter");
        }
        catch (IllegalArgumentException ex) {}

        try {
            Block_Grabber = HardwareMecanumBase.HardwareMap.get(DcMotor.class, "Block_Grabber");
        }
        catch (IllegalArgumentException ex) {}

        try {
            ElevatorClaw = HardwareMecanumBase.HardwareMap.get(DcMotor.class, "ElevatorClaw");
        }
        catch (IllegalArgumentException ex) {}
        try {
            ElevatorLifter = HardwareMecanumBase.HardwareMap.get(DcMotor.class, "ElevatorLifter");
        }
        catch (IllegalArgumentException ex) {}

        try {
            this.ExternalGyro = (ModernRoboticsI2cGyro)HardwareMecanumBase.HardwareMap.gyroSensor.get("gyro");
        }
        catch (IllegalArgumentException ex) {}

        try {
            this.Distance_Sensor = (Rev2mDistanceSensor)HardwareMecanumBase.HardwareMap.get(DistanceSensor.class, "distance");
        }
        catch (IllegalArgumentException ex) {}

        // Part of the rev hub, will always be present.
        this.IMU = HardwareMecanumBase.HardwareMap.get(BNO055IMU .class, "imu");
    }
}

