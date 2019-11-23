/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.GyroController;

public class GyroInitStateMachine {

    Telemetry telemetry;
    GyroInitStateMachine.RobotState state;
    GyroController gyro;

    enum RobotState
    {
        Start,
        Calibrating,
        Done
    }

    public void init(Telemetry telemetry, GyroController gyro) {
        this.gyro = gyro;
        gyro.StartCalibration();
        this.state = RobotState.Calibrating;

    }

    // Note that the Gyro MUST have this called in the primary run function's init loop to get the gyro ready.
    public boolean IsDone()
    {
        return (state == GyroInitStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        switch (state) {
            case Calibrating:
                if (!gyro.IsDoneCalibrating()) {
                    gyro.ResetZIndexIntegrator();
                    state = GyroInitStateMachine.RobotState.Done;
                }
                break;
        }
    }
}
