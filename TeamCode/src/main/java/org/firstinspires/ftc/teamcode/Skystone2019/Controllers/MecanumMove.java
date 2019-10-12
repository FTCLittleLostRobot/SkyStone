/* Little Lost Robots
   Core Devs: Danielle, Ryan
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Controllers;

import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumMove {

    public static final double GO_FORWARD = -1;
    public static final double GO_BACK = 1;
    public static final double GO_RIGHT = -1;
    public static final double GO_LEFT = 1;
    public enum RotationDirection{
        Right,
        Left
    }

    HardwareMecanumBase hwBase;

    int targetLeftFrontSpin = 0;
    int targetLeftFrontEncoderValue = 0;
    int targetRightFrontSpin = 0;
    int targetRightFrontEncoderValue = 0;
    int targetLeftBackSpin = 0;
    int targetLeftBackEncoderValue = 0;
    int targetRightBackSpin = 0;
    int targetRightBackEncoderValue = 0;

    public void init(HardwareMecanumBase hwBase){
        this.hwBase = hwBase;
    }

    public void StartMove(int speed, double inches, double x, double y, double rotation) {

        targetLeftFrontSpin = this.hwBase.GetWheelSpinDirection(HardwareMecanumBase.WheelControl.LeftFrontDrive,x,y,rotation);
        targetRightFrontSpin = this.hwBase.GetWheelSpinDirection(HardwareMecanumBase.WheelControl.RightFrontDrive,x,y,rotation);
        targetLeftBackSpin = this.hwBase.GetWheelSpinDirection(HardwareMecanumBase.WheelControl.LeftBackDrive,x,y,rotation);
        targetRightBackSpin = this.hwBase.GetWheelSpinDirection(HardwareMecanumBase.WheelControl.RightBackDrive,x,y,rotation);


        if (hwBase.left_front_drive != null){
            // Determine new target position, and pass to motor controller
            this.hwBase.left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.targetLeftFrontEncoderValue = this.hwBase.left_front_drive.getCurrentPosition() +
                    (targetLeftFrontSpin * (int) (inches * HardwareMecanumBase.WHEEL_COUNTS_PER_INCH));
        }
        if (hwBase.right_front_drive != null){
            // Determine new target position, and pass to motor controller
            this.hwBase.right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.targetRightFrontEncoderValue = this.hwBase.right_front_drive.getCurrentPosition() +
                    (targetRightFrontSpin * (int) (inches * HardwareMecanumBase.WHEEL_COUNTS_PER_INCH));
        }
        if (hwBase.left_back_drive != null){
            // Determine new target position, and pass to motor controller
            this.hwBase.left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.targetLeftBackEncoderValue = this.hwBase.left_back_drive.getCurrentPosition() +
                    (targetLeftBackSpin * (int) (inches * HardwareMecanumBase.WHEEL_COUNTS_PER_INCH));
        }
        if (hwBase.right_back_drive != null){
            // Determine new target position, and pass to motor controller
            this.hwBase.right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.targetRightBackEncoderValue = this.hwBase.right_back_drive.getCurrentPosition() +
                    (targetRightBackEncoderValue * (int) (inches * HardwareMecanumBase.WHEEL_COUNTS_PER_INCH));
        }

        this.hwBase.SpeedMultiplier = speed;
        this.hwBase.MoveMecanum(x,y,rotation);
    }



    public void StartRotate(Telemetry telemetry, int speed, double angle, RotationDirection direction) {
        telemetry.addData("Current State", direction.toString());

        int encoderTicks = 0;
        int targetLeftFrontRotation;

        if (angle == 90) {
            encoderTicks = 1560 / 2; //23
        }
        if (angle == 180) {
            encoderTicks = 1578;
        }

        if (direction == RotationDirection.Left){
            targetLeftFrontRotation = 1;
        }
        else {
            targetLeftFrontRotation = -1;
        }
        targetLeftFrontSpin = this.hwBase.GetWheelSpinDirection(HardwareMecanumBase.WheelControl.LeftFrontDrive,0,0, targetLeftFrontRotation);


        if (hwBase.left_front_drive != null){
            // Determine new target position, and pass to motor controller
            this.hwBase.left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hwBase.right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hwBase.left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hwBase.right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



            int newLeftFrontTarget = this.hwBase.left_front_drive.getCurrentPosition() + (targetLeftFrontSpin * encoderTicks);
            targetLeftFrontEncoderValue = newLeftFrontTarget;


        }

        this.hwBase.SpeedMultiplier = speed;
        this.hwBase.MoveMecanum(0,0, targetLeftFrontRotation);
    }

    public boolean IsDone() {
        boolean isWheelDone;
        boolean areAllWheelsDone = true;

        if (hwBase.left_front_drive != null){
            // this tells the robot if it is positive or negative
            if(targetLeftFrontSpin > 0){
                isWheelDone = this.hwBase.left_front_drive.getCurrentPosition() >= targetLeftFrontEncoderValue; // if it is positive
            }
            else {
                isWheelDone = this.hwBase.left_front_drive.getCurrentPosition() <= targetLeftFrontEncoderValue; // if it is negative
            }

            if (isWheelDone){
                hwBase.left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                areAllWheelsDone = false;
            }
        }

        if (hwBase.right_front_drive != null){
            // this tells the robot if it is positive or negative
            if(targetRightFrontSpin > 0){
                isWheelDone = this.hwBase.right_front_drive.getCurrentPosition() >= targetRightFrontEncoderValue; // if it is positive
            }
            else {
                isWheelDone = this.hwBase.right_front_drive.getCurrentPosition() <= targetRightFrontEncoderValue; // if it is negative
            }

            if (isWheelDone){
                hwBase.right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                areAllWheelsDone = false;
            }
        }

        if (hwBase.left_back_drive != null){
            // this tells the robot if it is positive or negative
            if(targetLeftBackSpin > 0){
                isWheelDone = this.hwBase.left_back_drive.getCurrentPosition() >= targetLeftBackEncoderValue; // if it is positive
            }
            else {
                isWheelDone = this.hwBase.left_back_drive.getCurrentPosition() <= targetLeftBackEncoderValue; // if it is negative
            }

            if (isWheelDone){
                hwBase.left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                areAllWheelsDone = false;
            }
        }

        if (hwBase.right_back_drive != null){
            // this tells the robot if it is positive or negative
            if(targetRightBackSpin > 0){
                isWheelDone = this.hwBase.right_back_drive.getCurrentPosition() >= targetRightBackEncoderValue; // if it is positive
            }
            else {
                isWheelDone = this.hwBase.right_back_drive.getCurrentPosition() <= targetRightBackEncoderValue; // if it is negative
            }

            if (isWheelDone){
                hwBase.right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                areAllWheelsDone = false;
            }
        }

        return areAllWheelsDone;
    }

    public void Complete() {
        // Stop all motion;
        this.hwBase.ResetMotors();
    }
}



