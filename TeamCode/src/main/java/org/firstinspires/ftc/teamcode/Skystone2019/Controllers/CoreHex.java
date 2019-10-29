/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Controllers;

import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
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


    public void init(HardwareMecanumBase hwBase, CoreHexMotors motor ) {
        if (motor == CoreHexMotors.BlockLifter){
            coreHex = hwBase.Block_Lifter;
        }
        else {
            coreHex = hwBase.Block_Grabber;
        }
    }


    public void Start(Telemetry telemetry, RotationDirection direction) {
        int newTarget = -1;

        if (direction == RotationDirection.Down){
            newTarget = -77;
        }
        else {
            newTarget = 0;
        }
        coreHex.setTargetPosition(newTarget);
        coreHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        coreHex.setPower(0.1);


        telemetry.addData("Current Position", coreHex.getCurrentPosition());
        telemetry.addData("New Target", newTarget);
    }

    public boolean IsDone() {
        return !(coreHex.isBusy());
    }

    public void Complete() {
        coreHex.setPower(0);
    }
}



