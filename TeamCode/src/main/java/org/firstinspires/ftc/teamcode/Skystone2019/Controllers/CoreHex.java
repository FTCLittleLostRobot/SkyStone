/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Controllers;

import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CoreHex {
    public enum RotationDirection {
        Down,
        Up

    }

    public enum CoreHexMotors{
        BlockLifter,
        BlockGrabber
    }
    DcMotor coreHex = null;
    CoreHexMotors motorType = null;
    public int newTarget = 0;
    public int startingPosition = 0;

    public void init(HardwareMecanumBase hwBase, CoreHexMotors motor ) {
        motorType = motor;
        if (motor == CoreHexMotors.BlockLifter){
            coreHex = hwBase.Block_Lifter;
        }
        else {
            coreHex = hwBase.Block_Grabber;
        }
        coreHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        coreHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ((DcMotorEx)coreHex).setTargetPositionTolerance(10);
    }


    public void Start(Telemetry telemetry, RotationDirection direction) {
        startingPosition = coreHex.getCurrentPosition();
        int directionOfMotor = 1;

        if (motorType == CoreHexMotors.BlockLifter) {
            if (direction == RotationDirection.Down) {
                newTarget = 0;
                directionOfMotor = -1;
                //coreHex.setDirection(DcMotorSimple.Direction.REVERSE);
            } else if (direction == RotationDirection.Up){
                newTarget = 30;

                //coreHex.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        if (motorType == CoreHexMotors.BlockGrabber) {
            if (direction == RotationDirection.Down) {
                newTarget = -205;
                directionOfMotor = -1;

                //coreHex.setDirection(DcMotorSimple.Direction.REVERSE);

            } else if (direction == RotationDirection.Up){
                newTarget = -130;
                //coreHex.setDirection(DcMotorSimple.Direction.FORWARD);

            }
        }

        if ( ((direction == RotationDirection.Up) && (startingPosition <= newTarget))
                || ((direction == RotationDirection.Down) && (startingPosition >= newTarget)) ){
            coreHex.setPower(0.8 * directionOfMotor);
        }
        else
        {
            coreHex.setPower(0);
        }

        telemetry.addData("Current Position", coreHex.getCurrentPosition());
        telemetry.addData("New Target", newTarget);
    }

    public boolean IsDone() {
        if (coreHex.getPower() != 0)
        {
            if (startingPosition >= newTarget){
                // down : starts at 100, target is -10
                if (coreHex.getCurrentPosition() <= newTarget){
                    return true;
                }
            }
            else if (startingPosition <= newTarget) {
                if (coreHex.getCurrentPosition() >= newTarget){
                    return true;
                }
            }
        }
        else
        {
            return true;
        }


        return false;
    }

    public void Complete() {
        coreHex.setPower(0);

    }
}
