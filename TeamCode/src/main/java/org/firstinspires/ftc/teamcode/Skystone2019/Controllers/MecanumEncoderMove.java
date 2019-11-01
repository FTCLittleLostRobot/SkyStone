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

    private int targetLeftFrontSpin = 0;
    public int targetLeftFrontEncoderValue = 0;

    private int targetRightFrontSpin = 0;
    public int targetRightFrontEncoderValue = 0;

    private int targetLeftBackSpin = 0;
    public int targetLeftBackEncoderValue = 0;

    private int targetRightBackSpin = 0;
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

        targetLeftFrontSpin = this.mecanumMotors.GetWheelSpinDirection(MecanumMotor.WheelControl.LeftFrontDrive,x,y,rotation);
        targetRightFrontSpin = this.mecanumMotors.GetWheelSpinDirection(MecanumMotor.WheelControl.RightFrontDrive,x,y,rotation);
        targetLeftBackSpin = this.mecanumMotors.GetWheelSpinDirection(MecanumMotor.WheelControl.LeftBackDrive,x,y,rotation);
        targetRightBackSpin = this.mecanumMotors.GetWheelSpinDirection(MecanumMotor.WheelControl.RightBackDrive,x,y,rotation);


        if (this.mecanumMotors.LeftFrontMotor != null){
            this.targetLeftFrontEncoderValue = this.mecanumMotors.LeftFrontMotor.getCurrentPosition() +
                    (targetLeftFrontSpin * (int) (inches * WHEEL_COUNTS_PER_INCH));
        }
        if (this.mecanumMotors.RightFrontMotor != null){
            this.targetRightFrontEncoderValue = this.mecanumMotors.RightFrontMotor.getCurrentPosition() +
                    (targetRightFrontSpin * (int) (inches * WHEEL_COUNTS_PER_INCH));
        }
        if (this.mecanumMotors.LeftBackMotor != null){
            this.targetLeftBackEncoderValue = this.mecanumMotors.LeftBackMotor.getCurrentPosition() +
                    (targetLeftBackSpin * (int) (inches * WHEEL_COUNTS_PER_INCH));
        }
        if (this.mecanumMotors.RightBackMotor != null){
            this.targetRightBackEncoderValue = this.mecanumMotors.RightBackMotor.getCurrentPosition() +
                    (targetRightBackSpin * (int) (inches * WHEEL_COUNTS_PER_INCH));
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
        telemetry.addData("Current State", direction.toString());

        this.useRunToPosition = false;

        int encoderTicks = 0;
        int targetLeftFrontRotation;

        if (angle == 90) {
            encoderTicks = 640 / 2; //23
        }
        if (angle == 180) {
            encoderTicks = 1280;
        }

        if (direction == RotationDirection.Left){
            targetLeftFrontRotation = 1;
        }
        else {
            targetLeftFrontRotation = -1;
        }
        targetLeftFrontSpin = mecanumMotors.GetWheelSpinDirection(MecanumMotor.WheelControl.LeftFrontDrive,0,0, targetLeftFrontRotation);


        if (this.mecanumMotors.LeftFrontMotor != null){

            int newLeftFrontTarget = this.mecanumMotors.LeftFrontMotor.getCurrentPosition() + (targetLeftFrontSpin * encoderTicks);
            targetLeftFrontEncoderValue = newLeftFrontTarget;
        }

        this.mecanumMotors.SetSpeedToValue(speed);
        this.mecanumMotors.MoveMecanum(0,0, targetLeftFrontRotation);
    }

    public boolean IsDone() {
        boolean isWheelDone;
        boolean areAllWheelsDone = true;

        if (!useRunToPosition) {
            if (this.mecanumMotors.LeftFrontMotor != null) {
                // this tells the robot if it is positive or negative
                if (targetLeftFrontSpin > 0) {
                    isWheelDone = this.mecanumMotors.LeftFrontMotor.getCurrentPosition() >= targetLeftFrontEncoderValue; // if it is positive
                } else {
                    isWheelDone = this.mecanumMotors.LeftFrontMotor.getCurrentPosition() <= targetLeftFrontEncoderValue; // if it is negative
                }

                if (isWheelDone) {
                    this.Complete();
                } else {
                    areAllWheelsDone = false;
                }
            }

            return areAllWheelsDone;
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
}



