/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.robot.RobotState;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumMoveStateMachine {

    Telemetry telemetry;
    MecanumEncoderMove moveRobot;
    MecanumMoveStateMachine.RobotState state;
    MecanumMotor motors;

    double inches = 0;
    double x = 0;
    double y = 0;
    double rotation = 0;

    enum RobotState
    {
        Start,
        Move,
        Moving,
        Done
    }

    public void init(Telemetry telemetry, MecanumMotor motors) {

        this.telemetry = telemetry;
        this.moveRobot = new MecanumEncoderMove();
        this.motors = motors;
        this.moveRobot.init(motors);

        state = MecanumMoveStateMachine.RobotState.Start;
    }

    public void Start(double inches, double x, double y, double rotation)
    {
        this.inches = inches;
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        state = MecanumMoveStateMachine.RobotState.Move;
    }

    public boolean IsDone()
    {
        return (state == MecanumMoveStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        telemetry.addData("Current State", state.toString());
        telemetry.addData("targetLeftFrontEncoderValue", this.moveRobot.targetLeftFrontEncoderValue);
        telemetry.addData("targetLeftFrontSpinValue", this.moveRobot.targetLeftFrontSpin);
        telemetry.addData("Current LeftFront Position", this.motors.LeftFrontMotor.getCurrentPosition());
        telemetry.addData("---", "");

        telemetry.addData("targetRightFrontEncoderValue", this.moveRobot.targetRightFrontEncoderValue);
        telemetry.addData("targetRightFrontSpinValue", this.moveRobot.targetRightFrontSpin);
        telemetry.addData("Current RightFront Position", this.motors.RightFrontMotor.getCurrentPosition());
        telemetry.addData("---", "");

        telemetry.addData("targetLeftBackEncoderValue", this.moveRobot.targetLeftBackEncoderValue);
        telemetry.addData("targetLeftBackSpinValue", this.moveRobot.targetLeftBackSpin);
        telemetry.addData("Current LeftBack Position", this.motors.LeftBackMotor.getCurrentPosition());
        telemetry.addData("---", "");

        telemetry.addData("targetRightBackEncoderValue", this.moveRobot.targetRightBackEncoderValue);
        telemetry.addData("targetRightBackSpinValue", this.moveRobot.targetRightBackSpin);
        telemetry.addData("Current RightBack Position", this.motors.RightBackMotor.getCurrentPosition());

        switch (state)
        {
            case Move:
                this.moveRobot.StartMove(10, inches, x, y, rotation);
                state = RobotState.Moving;
                break;

            case Moving:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.Done;
                }
                break;


            case Done:
                state = MecanumMoveStateMachine.RobotState.Done;
                break;
        }
    }
}
