/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;

public class GyroInitStateMachine {

    Telemetry telemetry;
    GyroInitStateMachine.RobotState state;
    ModernRoboticsI2cGyro gyro;

    enum RobotState
    {
        Calibrating,
        Done
    }

    public void init(Telemetry telemetry, ModernRoboticsI2cGyro gyro) {
        this.gyro = gyro;
        gyro.calibrate();
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
                if (!gyro.isCalibrating()) {
                    gyro.resetZAxisIntegrator();
                    state = GyroInitStateMachine.RobotState.Done;
                }
                break;
        }
    }
}
