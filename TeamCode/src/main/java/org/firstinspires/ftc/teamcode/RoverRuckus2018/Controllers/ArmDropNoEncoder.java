/* Little Lost Robots
   Core Devs: Danielle
*/

//Core Devs: Danielle

package org.firstinspires.ftc.teamcode.RoverRuckus2018.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoverRuckus2018.HardwareMecanumBase;

public class ArmDropNoEncoder {

    HardwareMecanumBase hwBase;
    Telemetry telemetry;
    //public Servo servo = null;

    public void init(HardwareMecanumBase hwBase, Telemetry telemetry, HardwareMap hardwareMap){

        this.hwBase = hwBase;
        this.telemetry = telemetry;
       // servo = hardwareMap.servo.get("servo");

    }

    public void ArmDropLeft( ) {
       if (hwBase.ArmDropLeft != null ) {
           hwBase.ArmDropLeft.setPower(0.1);
           telemetry.addData(" ArmDrop", "Left Arm is moving");
           telemetry.update();
        }
    }

    public void ArmDropRight() {
        if (hwBase.ArmDropRight != null ) {
            hwBase.ArmDropRight.setPower(0.1);
            telemetry.addData("ArmDrop", "Right Arm is moving");
            telemetry.update();
        }
    }


    public void Complete() {
        // Stop all motion;
        if (hwBase.ArmDropLeft != null)
        {
            hwBase.ArmDropLeft.setPower(0);
            telemetry.addData("ArmDropLeft", " left Arm drop is complete");

        }
    }

}
