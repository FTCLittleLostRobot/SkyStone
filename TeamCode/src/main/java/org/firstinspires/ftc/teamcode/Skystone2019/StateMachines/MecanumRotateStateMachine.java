/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;


public class MecanumRotateStateMachine {

    Telemetry telemetry;
    MecanumMotor motors;
    MecanumEncoderMove moveRobot;
    ModernRoboticsI2cGyro gyro;

    Double degrees;
    int speed = 20;
    MecanumRotateStateMachine.RobotState state;
    ElapsedTime holdTimer = new ElapsedTime();


    enum RobotState
    {
        Start,
        StartEncoderRotate,
        CheckEncoderRotate,
        StartGyroRotation,
        StartGyroHold,
        Done
    }

    public void init(Telemetry telemetry, MecanumMotor motors, ModernRoboticsI2cGyro gyro)
    {

        this.telemetry = telemetry;
        this.motors = motors;
        this.moveRobot = new MecanumEncoderMove();
        this.moveRobot.init(motors);

        // For Gyro Support
        this.gyro = gyro;
        motors.SetMecanumBreak();

        state = MecanumRotateStateMachine.RobotState.Start;
    }

    // SPeed is not used when moving via Encoders
    public void Start(Double degrees, int speed)
    {
        this.degrees= degrees;
        this.speed = speed;
        state = MecanumRotateStateMachine.RobotState.StartEncoderRotate;
    }

    public boolean IsDone()
    {
        return (state == MecanumRotateStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        telemetry.addData("Current State", state.toString());

        switch (state)
        {
            case StartEncoderRotate:
                this.moveRobot.StartRotate(telemetry, 25, degrees, MecanumEncoderMove.RotationDirection.Right );
                state = RobotState.CheckEncoderRotate;
                break;

            case CheckEncoderRotate:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.Done;
                }
                break;

            case StartGyroRotation:

                if (this.motors.MovingToHeadingGyro(speed, this.degrees, .1, gyro, 5)){
                    holdTimer.reset();
                    state = RobotState.StartGyroHold;
                }

                break;

            case StartGyroHold:
                if ((holdTimer.time() <= 2)) {
                    // Update telemetry & Allow time for other processes to run.
                    motors.MovingToHeadingGyro(10, this.degrees, .1, gyro, 0);
                }
                else {
                    state = MecanumRotateStateMachine.RobotState.Done;
                }
                break;

            case Done:
                state = MecanumRotateStateMachine.RobotState.Done;
                break;
        }
    }
}
