/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Controllers;

import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

public class CoreHex {
    public enum RotationDirection {
        Down,
        Up,
        DropBlockLifter,
        DropBlockGrabber,
        PositionZeroGrabber,
        ControlledUp,
        ControlledDown
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
            } else if (direction == RotationDirection.Up) {
                newTarget = 50;
            } else if (direction == RotationDirection.DropBlockLifter){
                directionOfMotor = -1;
                newTarget = 35;
            }
        }

        if (motorType == CoreHexMotors.BlockGrabber) {
            if (direction == RotationDirection.Down) {
                newTarget = -205;
                directionOfMotor = -1;
            } else if (direction == RotationDirection.Up) {
                newTarget = -90; // 130
            } else if (direction == RotationDirection.ControlledUp){
                newTarget = coreHex.getCurrentPosition() + 5;
            } else if (direction == RotationDirection.ControlledDown){
                newTarget = coreHex.getCurrentPosition() - 5;
                directionOfMotor = -1;
            } else if (direction == RotationDirection.DropBlockGrabber){
                newTarget = -100;
            } else if (direction == RotationDirection.PositionZeroGrabber){
                newTarget = 0;
                directionOfMotor = 1;
            }

        }

        if ( ((direction == RotationDirection.Up) && (startingPosition <= newTarget))
                || ((direction == RotationDirection.Down) && (startingPosition >= newTarget))
                || (direction == RotationDirection.ControlledDown && (startingPosition >= newTarget))
                || (direction == RotationDirection.ControlledUp) && (startingPosition <= newTarget)
                || (direction == RotationDirection.DropBlockLifter) && (startingPosition >= newTarget)
                || (direction == RotationDirection.DropBlockGrabber) && (startingPosition <= newTarget)
                || (direction == RotationDirection.PositionZeroGrabber) && (startingPosition <= newTarget)){
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
