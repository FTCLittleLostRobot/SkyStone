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
        Right,
        Left
    }

    DcMotor coreHex = null;
    int startingPos = -1;
    int newTarget = -1;

    public void init(HardwareMecanumBase hwBase) {
        coreHex = hwBase.Core_Hex;
        startingPos = coreHex.getCurrentPosition();
        coreHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void Start(Telemetry telemetry, RotationDirection direction) {
        int newTarget = -1;

        if (direction == RotationDirection.Right){
            coreHex.setDirection(DcMotor.Direction.REVERSE);
        }
        else {
            coreHex.setDirection(DcMotor.Direction.FORWARD);
        }
        newTarget = coreHex.getCurrentPosition() + 63;
        coreHex.setTargetPosition(newTarget);
        coreHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        coreHex.setPower(0.1);


        telemetry.addData("Starting Position", startingPos);
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



