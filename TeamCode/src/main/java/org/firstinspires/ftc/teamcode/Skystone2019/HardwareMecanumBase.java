/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    public enum WheelControl {
        LeftFrontDrive,
        RightFrontDrive,
        LeftBackDrive,
        RightBackDrive,
    }

    /* Public OpMode members. */

    public DcMotor left_front_drive = null;
    public DcMotor right_front_drive = null;
    public DcMotor left_back_drive = null;
    public DcMotor right_back_drive = null;


    /* local OpMode members. */
    public HardwareMap hardwareMap = null;
    private static boolean ONBOT_ACTIVE = true;
    private static final double COUNTS_PER_MOTOR_REV = 400.6;  // eg: Countable events per revolution of Output shaft
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double LIFT_GEAR_REDUCTION = 2.0;     // This is < 2.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double LIFT_DIAMETER_INCHES = 0.4;     // For figuring circumference
    public static final double WHEEL_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double LIFT_COUNTS_PER_INCH = 8000;
    //public static final double LIFT_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //(LIFT_DIAMETER_INCHES * 3.1415);

    private ElapsedTime period = new ElapsedTime();
    public int SpeedMultiplier = 50;

    /* Constructor *private HardwareMecanumBase2018(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        // i changed the front two motors fron Reverse to Fowards or Fowards to reverse
        // Define and Initialize Motors
        /*
        todo: If code is can't find motor then add back in and use Android Studio, TryGet WILL NOT WORK WITH ONBOT
        left_front_drive = hardwareMap.tryGet(DcMotor.class, "left_front");
        right_front_drive = hardwareMap.tryGet(DcMotor.class, "right_front");
        left_back_drive = hardwareMap.tryGet(DcMotor.class, "left_back");
        right_back_drive = hardwareMap.tryGet(DcMotor.class, "right_back");
        */
        left_front_drive = hardwareMap.get(DcMotor.class, "left_front");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back");

        if (left_front_drive != null) {
            left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        }

        if (right_front_drive != null){
            right_front_drive.setDirection(DcMotor.Direction.REVERSE);

        }

        if (left_back_drive != null) {
            left_back_drive.setDirection(DcMotor.Direction.FORWARD);
        }

        if (right_back_drive != null) {
            right_back_drive.setDirection(DcMotor.Direction.REVERSE);
        }

        ResetMotors();
    }

    public void ResetMotors() {
        // Set all motors to zero power
        if (right_back_drive != null) {
            right_back_drive.setPower(0);
            right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (left_back_drive != null) {
            left_back_drive.setPower(0);
            left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (right_front_drive != null) {
            right_front_drive.setPower(0);
            right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (left_front_drive != null) {
            left_front_drive.setPower(0);
            left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void DrivePower(WheelControl wheel, double power) {

        switch (wheel) {
            case LeftBackDrive:
                left_back_drive.setPower(power * ((double) SpeedMultiplier / 100));
                break;
            case RightBackDrive:
                right_back_drive.setPower(power * ((double) SpeedMultiplier / 100));
                break;
            case LeftFrontDrive:
                left_front_drive.setPower(power * ((double) SpeedMultiplier / 100));
                break;
            case RightFrontDrive:
                right_front_drive.setPower(power * ((double) SpeedMultiplier / 100));
                break;

            default:
                break;

        }
    }

    public int GetWheelSpinDirection(WheelControl wheel, double x, double y, double rotation)
    {
        double r = Math.hypot( x, y);
        double robotAngle = Math.atan2(y,x) - Math.PI / 4;
        double v = 0;

        switch (wheel)
        {
 /*
            case LeftFrontDrive:  //Julia 12-1-6:35 switch left front with right front
                v = r * Math.sin(robotAngle) - rotation;
                break;
            case RightFrontDrive:
                v = r * Math.cos(robotAngle) + rotation;
                break;
*/
            case LeftFrontDrive:
                v = r * Math.cos(robotAngle) + rotation;
                break;

            case RightFrontDrive:
                v = r * Math.sin(robotAngle) - rotation;
                break;

            case LeftBackDrive:
                v = r * Math.sin(robotAngle) + rotation;
                break;

            case RightBackDrive:
                v = r * Math.sin(robotAngle) - rotation;
                break;


        }

        if (v >= 0) {
            return 1;
        }
        else {
            return -1;
        }
    }

    public void MoveMecanum(double x, double y, double rotation)
    {
        double r = Math.hypot( x, y);
        double robotAngle = Math.atan2(y,x) - Math.PI / 4;

        final double v1 = r * Math.cos(robotAngle) + rotation;
        final double v2 = r * Math.sin(robotAngle) - rotation;
        final double v3 = r * Math.sin(robotAngle) + rotation;
        final double v4 = r * Math.cos(robotAngle) - rotation;

        //DrivePower(WheelControl.LeftFrontDrive, v2);  //Julia 12-1-6:35 switch left front with right front
        //DrivePower(WheelControl.RightFrontDrive, v1);
        DrivePower(WheelControl.LeftFrontDrive, v1);
        DrivePower(WheelControl.RightFrontDrive, v2);
        DrivePower(WheelControl.LeftBackDrive,v3);
        DrivePower(WheelControl.RightBackDrive, v4);
    }



    public void IncreaseSpeed()
    {
        SpeedMultiplier = 70;
    } // high speed

    public void DecreaseSpeed()
    {
        SpeedMultiplier = 20;
    } // low speed

    public void ResetSpeed()
    {
        SpeedMultiplier = 50;
    }// main speed
}

