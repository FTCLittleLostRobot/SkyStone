/* Little Lost Robots
   Core Devs: Danielle, Ryan
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Controllers;

import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumEncoderMove {


    public static final double GO_FORWARD = -1;
    public static final double GO_BACK = 1;
    public static final double GO_RIGHT = -1;
    public static final double GO_LEFT = 1;

    public enum RotationDirection{
        Right,
        Left
    }

    private static final double COUNTS_PER_MOTOR_REV = 400.6;  // eg: Countable events per revolution of Output shaft
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private  static final double WHEEL_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private MecanumMotor mecanumMotors;
    private Boolean useRunToPosition = true;
    private int targetPositionTolerance = 30;

    public double targetLeftFrontPower = 0;
    public int targetLeftFrontSpin = 0;
    public int targetLeftFrontEncoderValue = 0;

    public double targetRightFrontPower = 0;
    public int targetRightFrontSpin = 0;
    public int targetRightFrontEncoderValue = 0;

    public double targetLeftBackPower = 0;
    public int targetLeftBackSpin = 0;
    public int targetLeftBackEncoderValue = 0;

    public double targetRightBackPower = 0;
    public int targetRightBackSpin = 0;
    public int targetRightBackEncoderValue = 0;

    public void init(MecanumMotor mMotors){

        this.mecanumMotors = mMotors;

        if (this.mecanumMotors.LeftFrontMotor != null)
        {
            mecanumMotors.LeftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecanumMotors.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanumMotors.LeftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ((DcMotorEx)mecanumMotors.LeftFrontMotor).setTargetPositionTolerance(targetPositionTolerance);
        }

        if (this.mecanumMotors.RightFrontMotor != null)
        {
            mecanumMotors.RightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecanumMotors.RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanumMotors.RightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ((DcMotorEx)mecanumMotors.RightFrontMotor).setTargetPositionTolerance(targetPositionTolerance);
        }

        if (this.mecanumMotors.LeftBackMotor != null)
        {
            mecanumMotors.LeftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecanumMotors.LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanumMotors.LeftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ((DcMotorEx)mecanumMotors.LeftBackMotor).setTargetPositionTolerance(targetPositionTolerance);
        }

        if (this.mecanumMotors.RightBackMotor != null)
        {
            mecanumMotors.RightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecanumMotors.RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanumMotors.RightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ((DcMotorEx)mecanumMotors.RightBackMotor).setTargetPositionTolerance(targetPositionTolerance);
        }
    }

    public void StartMove(int speed, double inches, double x, double y, double rotation ) {

        if (x != 0 && y != 0)
        {
            useRunToPosition = false;
        }
        else
        {
            useRunToPosition = true;
        }

        targetLeftFrontPower = this.mecanumMotors.CalculateMecanumPosition(MecanumMotor.WheelControl.LeftFrontDrive,x,y,rotation);
        targetRightFrontPower = this.mecanumMotors.CalculateMecanumPosition(MecanumMotor.WheelControl.RightFrontDrive,x,y,rotation);
        targetLeftBackPower = this.mecanumMotors.CalculateMecanumPosition(MecanumMotor.WheelControl.LeftBackDrive,x,y,rotation);
        targetRightBackPower = this.mecanumMotors.CalculateMecanumPosition(MecanumMotor.WheelControl.RightBackDrive,x,y,rotation);

        if (this.mecanumMotors.LeftFrontMotor != null){
            this.targetLeftFrontEncoderValue = this.mecanumMotors.LeftFrontMotor.getCurrentPosition() +
                    (int) (targetLeftFrontPower * (inches * WHEEL_COUNTS_PER_INCH));
        }
        if (this.mecanumMotors.RightFrontMotor != null){
            this.targetRightFrontEncoderValue = this.mecanumMotors.RightFrontMotor.getCurrentPosition() +
                    (int) (targetRightFrontPower * (inches * WHEEL_COUNTS_PER_INCH));
        }
        if (this.mecanumMotors.LeftBackMotor != null){
            this.targetLeftBackEncoderValue = this.mecanumMotors.LeftBackMotor.getCurrentPosition() +
                    (int) (targetLeftBackPower * (inches * WHEEL_COUNTS_PER_INCH));
        }
        if (this.mecanumMotors.RightBackMotor != null){
            this.targetRightBackEncoderValue = this.mecanumMotors.RightBackMotor.getCurrentPosition() +
                    (int) (targetRightBackPower * (inches * WHEEL_COUNTS_PER_INCH));
        }

        if (useRunToPosition)
        {
            this.mecanumMotors.PrepareRunMotorToPosition(this.mecanumMotors.LeftFrontMotor, this.targetLeftFrontEncoderValue);
            this.mecanumMotors.PrepareRunMotorToPosition(this.mecanumMotors.RightFrontMotor, this.targetRightFrontEncoderValue);
            this.mecanumMotors.PrepareRunMotorToPosition(this.mecanumMotors.LeftBackMotor, this.targetLeftBackEncoderValue);
            this.mecanumMotors.PrepareRunMotorToPosition(this.mecanumMotors.RightBackMotor, this.targetRightBackEncoderValue);

            this.mecanumMotors.RunMotorToPosition(this.mecanumMotors.LeftFrontMotor,  .5);
            this.mecanumMotors.RunMotorToPosition(this.mecanumMotors.RightFrontMotor, .5);
            this.mecanumMotors.RunMotorToPosition(this.mecanumMotors.LeftBackMotor, .5);
            this.mecanumMotors.RunMotorToPosition(this.mecanumMotors.RightBackMotor, .5);
        }
        else {
            this.mecanumMotors.SetSpeedToValue(speed);
            this.mecanumMotors.MoveMecanum(x, y, rotation);
        }
    }


    public void StartRotate(Telemetry telemetry, int speed, double angle, RotationDirection direction) {
        int encoderTicks = 0;

        if (angle == 90 && direction == RotationDirection.Right) {
            encoderTicks = 640;
        }
        else if (angle == 90 && direction == RotationDirection.Left){
            encoderTicks = 675;
        }
        else if (angle == 180) {
            encoderTicks = 1325;
        }
        else if (angle == 45) {
            encoderTicks = 250;
        }

        StartRotateDirectEncoderTicks(telemetry, speed, encoderTicks, direction);
    }

    public void StartRotateDirectEncoderTicks(Telemetry telemetry, int speed, int encoderTicks, RotationDirection direction) {
        telemetry.addData("Current State", direction.toString());

        this.useRunToPosition = false;
        targetLeftFrontSpin = 0;
        targetRightFrontSpin = 0;
        targetLeftBackSpin = 0;
        targetRightBackSpin = 0;

        int targetLeftFrontRotation;

        if (direction == RotationDirection.Left){
            targetLeftFrontRotation = 1;
        }
        else {
            targetLeftFrontRotation = -1;
        }

        targetLeftFrontPower = mecanumMotors.CalculateMecanumPosition(MecanumMotor.WheelControl.LeftFrontDrive,0,0, targetLeftFrontRotation);
        if (this.mecanumMotors.LeftFrontMotor != null){
            targetLeftFrontEncoderValue = this.mecanumMotors.LeftFrontMotor.getCurrentPosition() + (targetLeftFrontRotation * encoderTicks);
        }

        this.mecanumMotors.SetSpeedToValue(speed);
        this.mecanumMotors.MoveMecanum(0,0, targetLeftFrontRotation);
    }

    private boolean IsWheelDone(DcMotor motor, double spinDirection, int targetEncoderValue)
    {
        if (motor != null)
        {
            if (spinDirection > 0.0)
                return motor.getCurrentPosition() >= targetEncoderValue;
            else
                return motor.getCurrentPosition() <= targetEncoderValue;
        }
        else
            return true;
    }

    public boolean IsDone() {
        boolean areAllWheelsDone = true;

        if (!useRunToPosition) {
            if (Math.abs(targetLeftFrontPower) > Math.abs(targetRightFrontPower))
            {
                areAllWheelsDone = IsWheelDone(this.mecanumMotors.LeftFrontMotor, targetLeftFrontPower, targetLeftFrontEncoderValue);
            }
            else
            {
                areAllWheelsDone = IsWheelDone(this.mecanumMotors.RightFrontMotor, targetRightFrontPower, targetRightFrontEncoderValue);
            }

            if (areAllWheelsDone)
            {
                return true;
            } else { return false; }
        }
        else
        {
            return !this.mecanumMotors.LeftFrontMotor.isBusy() &&
                    !this.mecanumMotors.RightFrontMotor.isBusy() &&
                    !this.mecanumMotors.LeftBackMotor.isBusy() &&
                    !this.mecanumMotors.RightBackMotor.isBusy();
        }
    }

    public void Complete() {
        // Stop all motion;
        this.mecanumMotors.ResetMotors();

    }

    public int GetLeftMotorEncodePosition()
    {
        return this.mecanumMotors.LeftFrontMotor.getCurrentPosition();
    }
}



