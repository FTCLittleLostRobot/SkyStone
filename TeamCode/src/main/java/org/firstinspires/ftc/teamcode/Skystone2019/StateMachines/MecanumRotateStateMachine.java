/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.GyroController;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;


public class MecanumRotateStateMachine {

    Telemetry telemetry;
    MecanumMotor motors;
    MecanumEncoderMove moveRobot;
    GyroController gyro;

    Double degrees;
    int origSpeed = 50;
    int speed = 20;
    MecanumRotateStateMachine.RobotState state;
    ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


    enum RobotState
    {
        Start,
        StartEncoderRotate,
        CheckEncoderRotate,

        StartGyroRotation,
        WaitForGyroRotate,
        StartGyroHold,
        Done
    }

    public void init(Telemetry telemetry, MecanumMotor motors, GyroController gyro)
    {
        this.telemetry = telemetry;
        this.motors = motors;
        this.moveRobot = new MecanumEncoderMove();
        this.moveRobot.init(motors);

        // For Gyro Support
        this.gyro = gyro;

        state = MecanumRotateStateMachine.RobotState.Start;
    }

    // SPeed is not used when moving via Encoders
    public void StartWithEncoder(Double degrees)
    {
        this.degrees= degrees;
        this.speed = 50;
        state = MecanumRotateStateMachine.RobotState.StartEncoderRotate;
    }

    public void StartWithGyro(Double degrees, int speed)
    {
        this.degrees= degrees;
        this.speed = speed;
        motors.SetMecanumBreak();
        state = RobotState.StartGyroRotation;
    }

    public boolean IsDone()
    {
        return (state == MecanumRotateStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        telemetry.addData("Current State", state.toString());
        double error;
        double steer;

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

                // Before we rotate, save the current speed multiplier and update it to what user wants
                this.origSpeed = this.motors.GetSpeedMultiplier();
                this.motors.SetSpeedToValue(speed);

                // determine turn power based on +/- error
                error = gyro.GetErrorToTarget(this.degrees);

                // Get turning force and rotate
                steer = gyro.GetSteeringForce(error, .1);
                this.motors.MoveMecanum(0,0, steer);

                state = RobotState.WaitForGyroRotate;

                break;
            case WaitForGyroRotate:

                // determine turn power based on +/- error
                error = gyro.GetErrorToTarget(this.degrees);

                if (Math.abs(error) <= 10) {
                    // If we are close enough, change to hold on time.
                    holdTimer.reset();
                    state = RobotState.StartGyroHold;
                }
                else
                {
                    // If not close enough, then find out what the steer force we should
                    // get and continue moving.
                    steer = gyro.GetSteeringForce(error, .1);
                    this.motors.MoveMecanum(0,0, steer);
                }

                break;

            case StartGyroHold:
                if ((holdTimer.time() <= 2)) {
                    this.motors.SetSpeedToValue(15);

                    error = gyro.GetErrorToTarget(this.degrees);
                    steer = gyro.GetSteeringForce(error, .8);
                    this.motors.MoveMecanum(0,0, steer);
                }
                else {
                    // Reset the motors
                    this.motors.MoveMecanum(0,0, 0);
                    this.motors.ResetMotors();

                    // Before we finish, reset the motor speed to prevent other robot actions
                    // from using the updated speed value
                    this.motors.SetSpeedToValue(origSpeed);

                    state = RobotState.Done;
                }
                break;
        }
    }
}
