package org.firstinspires.ftc.teamcode.Skystone2019.Controllers;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumMotor {

    public enum WheelControl {
        LeftFrontDrive,
        RightFrontDrive,
        LeftBackDrive,
        RightBackDrive
    }

    public DcMotor LeftFrontMotor = null;
    public DcMotor RightFrontMotor = null;
    public DcMotor LeftBackMotor = null;
    public DcMotor RightBackMotor = null;

    private HardwareMecanumBase hardwareBase = null;

    private double LeftFrontMotorPowerAdjustment = 1;
    private double RightFrontMotorPowerAdjustment = 1.1;
    private double LeftBackMotorPowerAdjustment = 1.1;
    private double RightBackMotorPowerAdjustment = 1;

    private int SpeedMultiplier = 50;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMecanumBase hwBase) {

        hardwareBase = hwBase;
        this.LeftFrontMotor = this.hardwareBase.left_front_drive;
        this.RightFrontMotor = this.hardwareBase.right_front_drive;
        this.LeftBackMotor = this.hardwareBase.left_back_drive;
        this.RightBackMotor = this.hardwareBase.right_back_drive;

        if (this.LeftFrontMotor != null) {
            this.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.LeftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        if (this.RightFrontMotor != null) {
            this.RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.RightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if (this.LeftBackMotor != null) {
            this.LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.LeftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        if (this.RightBackMotor != null)
        {
            this.RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.RightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        ResetMotors();
    }

    public void ResetMotors() {
        // Set all motors to zero power
        if (this.RightBackMotor != null) {
            this.RightBackMotor.setPower(0);
        }

        if (this.LeftBackMotor != null) {
            this.LeftBackMotor.setPower(0);
        }

        if (this.RightFrontMotor != null) {
            this.RightFrontMotor.setPower(0);
        }

        if (this.LeftFrontMotor != null) {
            this.LeftFrontMotor.setPower(0);
        }

        this.RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void DrivePower(WheelControl wheel, double power) {

        switch (wheel) {
            case LeftFrontDrive:
                if (this.LeftFrontMotor != null)
                    this.LeftFrontMotor.setPower(power * ((double) SpeedMultiplier / 100));
                break;
            case RightFrontDrive:
                if (this.RightFrontMotor != null)
                    this.RightFrontMotor.setPower(power * ((double) SpeedMultiplier / 100));
                break;
            case LeftBackDrive:
                if (this.LeftBackMotor != null)
                    this.LeftBackMotor.setPower(power * ((double) SpeedMultiplier / 100));
                break;
            case RightBackDrive:
                if (this.RightBackMotor != null)
                    this.RightBackMotor.setPower(power * ((double) SpeedMultiplier / 100));
                break;
            default:
                break;

        }
    }

    public void PrepareRunMotorToPosition(DcMotor motor, int newTargetPosition)
    {
        motor.setTargetPosition(newTargetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void RunMotorToPosition(DcMotor motor, double power)
    {
        motor.setPower(power);
    }

    public int GetWheelSpinDirection(WheelControl wheel, double x, double y, double rotation)
    {
        double v = CalculateMecanumPosition(wheel, x, y, rotation);

        if (v > 0.001)
            return 1;
        else if (v < -.001)
            return -1;
        else
            return 0;
    }

    public void MoveMecanum(double x, double y, double rotation)
    {
        DrivePower(WheelControl.LeftFrontDrive,
                CalculateMecanumPosition(WheelControl.LeftFrontDrive, x, y, rotation) * LeftFrontMotorPowerAdjustment);

        DrivePower(WheelControl.RightFrontDrive,
                CalculateMecanumPosition(WheelControl.RightFrontDrive, x, y, rotation) * RightFrontMotorPowerAdjustment);

        DrivePower(WheelControl.LeftBackDrive,
                CalculateMecanumPosition(WheelControl.LeftBackDrive, x, y, rotation) * LeftBackMotorPowerAdjustment);

        DrivePower(WheelControl.RightBackDrive,
                CalculateMecanumPosition(WheelControl.RightBackDrive, x, y, rotation) * RightBackMotorPowerAdjustment);
    }

    public double CalculateMecanumPosition(WheelControl wheel, double x, double y, double rotation)
    {
        double r = Math.hypot( x, y);
        double robotAngle = Math.atan2(y,x) - Math.PI / (double)4;
        double v = 0;

        switch (wheel)
        {
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
                v = r * Math.cos(robotAngle) - rotation;
                break;
        }

        return v;
    }

    public int GetSpeedMultiplier()
    {
        return this.SpeedMultiplier;
    }

    public void SetSpeedToValue(int speed)
    {
        this.SpeedMultiplier = speed;
    }

    public void IncreaseSpeed()
    {
        this.SpeedMultiplier = 70;
    } // high speed

    public void DecreaseSpeed()
    {
        this.SpeedMultiplier = 20;
    } // low speed

    public void ResetSpeed()
    {
        this.SpeedMultiplier = 50;
    }// main speed
    public void HighestSpeed(){this.SpeedMultiplier = 80;}

    public void SetMecanumBreak (){

        this.LeftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.LeftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
