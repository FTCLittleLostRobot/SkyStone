/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumGyroRotateStateMachine {

    Telemetry telemetry;
    MecanumEncoderMove moveRobot;
    MecanumGyroRotateStateMachine.RobotState state;
    MecanumMotor motors;
    ModernRoboticsI2cGyro gyro;
    int speed;
    double angle;
    ElapsedTime holdTimer = new ElapsedTime();

    enum RobotState
    {
        ReadyForStart,
        StartRotation,
        StartHold,
        Done
    }

    public void init(Telemetry telemetry, MecanumMotor motors, ModernRoboticsI2cGyro gyro) {
        this.motors = motors;
        this.telemetry = telemetry;
        this.gyro = gyro;
        motors.SetMecanumBreak();
        this.state = RobotState.ReadyForStart;

    }

    public void Start(int speed, double angle)
    {
        this.speed = speed;
        this.angle = angle;
        state = MecanumGyroRotateStateMachine.RobotState.StartRotation;
    }

    public boolean IsDone()
    {
        return (state == MecanumGyroRotateStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        switch (state)
        {
            case StartRotation:

                if (motors.MovingToHeadingGyro(speed, angle, .1, gyro, 5)){
                    holdTimer.reset();
                    state = RobotState.StartHold;
                }

                break;

            case StartHold:
                if ((holdTimer.time() <= 2)) {
                    // Update telemetry & Allow time for other processes to run.
                    motors.MovingToHeadingGyro(10, angle, .1, gyro, 0);
                 }
                else {
                    state = MecanumGyroRotateStateMachine.RobotState.Done;
                }
                break;

            case Done:
                state = MecanumGyroRotateStateMachine.RobotState.Done;
                break;
        }

    }
}
