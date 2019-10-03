/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Controllers;

import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
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

    int targetSpin = 0;
    int targetEncoderValue = 0;
    int targetRotation = 0;


    public void init(HardwareMecanumBase hwBase){
        this.hwBase = hwBase;
    }

    public void StartMove(int speed, double inches, double x, double y, double rotation) {

        targetSpin  = this.hwBase.GetWheelSpinDirection(HardwareMecanumBase.WheelControl.LeftFrontDrive,x,y,rotation);


        if (hwBase.left_front_drive != null){
            // Determine new target position, and pass to motor controller
            this.hwBase.left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hwBase.right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hwBase.left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hwBase.right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            int newLeftFrontTarget = this.hwBase.left_front_drive.getCurrentPosition() +
                    (targetSpin * (int) (inches * HardwareMecanumBase.WHEEL_COUNTS_PER_INCH));
            targetEncoderValue = newLeftFrontTarget;


        }

        this.hwBase.SpeedMultiplier = speed;
        this.hwBase.MoveMecanum(x,y,rotation);
    }



    public void StartRotate(Telemetry telemetry, int speed, double angle, RotationDirection direction) {
        telemetry.addData("Current State", direction.toString());

        int encoderTicks = 0;

        if (angle == 90) {
            encoderTicks = 1560 / 2; //23
        }
        if (angle == 180) {
            encoderTicks = 1578;
        }

        if (direction == RotationDirection.Left){
            targetRotation = 1;
        }
        else {
            targetRotation = -1;
        }
        targetSpin  = this.hwBase.GetWheelSpinDirection(HardwareMecanumBase.WheelControl.LeftFrontDrive,0,0,targetRotation);


        if (hwBase.left_front_drive != null){
            // Determine new target position, and pass to motor controller
            this.hwBase.left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hwBase.right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hwBase.left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.hwBase.right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



            int newLeftFrontTarget = this.hwBase.left_front_drive.getCurrentPosition() + (targetSpin * encoderTicks);
            targetEncoderValue = newLeftFrontTarget;


        }

        this.hwBase.SpeedMultiplier = speed;
        this.hwBase.MoveMecanum(0,0, targetRotation);
    }

    public boolean IsDone() {
        boolean isDone;

        if (hwBase.left_front_drive != null){
            // this tells the robot if it is positive or negative
            if(targetSpin > 0){
                isDone = this.hwBase.left_front_drive.getCurrentPosition() >= targetEncoderValue; // if it is positive
            }
            else {
                isDone = this.hwBase.left_front_drive.getCurrentPosition() <= targetEncoderValue; // if it is negative
            }

            if (targetSpin > 0) {
                isDone = this.hwBase.left_front_drive.getCurrentPosition() >= targetEncoderValue; // if it is positive
            } else {
                isDone = this.hwBase.left_front_drive.getCurrentPosition() <= targetEncoderValue; // if it is negative
            }
        }
        else {
            isDone = true;
        }
        return isDone;
    }

    public void Complete() {
        // Stop all motion;
        this.hwBase.ResetMotors();
    }
}



