/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMove;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumMoveStateMachine {

    Telemetry telemetry;
    MecanumMove moveRobot;
    MecanumMoveStateMachine.RobotState state;
    HardwareMecanumBase robot;




    enum RobotState
    {
        Start,
        Move,
        Moving,
        Done
    }

    public void init(Telemetry telemetry, HardwareMecanumBase robot) {

        this.telemetry = telemetry;
        this.moveRobot = new MecanumMove();
        this.robot = robot;
        this.moveRobot.init(robot);

        state = MecanumMoveStateMachine.RobotState.Start;
    }

    public void Start()
    {
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
        telemetry.addData("Current LeftFront Position", this.robot.left_front_drive.getCurrentPosition());

        telemetry.addData("targetRightFrontEncoderValue", this.moveRobot.targetRightFrontEncoderValue);
        telemetry.addData("Current RightFront Position", this.robot.right_front_drive.getCurrentPosition());

        telemetry.addData("targetLeftBackEncoderValue", this.moveRobot.targetLeftBackEncoderValue);
        telemetry.addData("Current LeftBack Position", this.robot.left_back_drive.getCurrentPosition());

        telemetry.addData("targetRightBackEncoderValue", this.moveRobot.targetRightBackEncoderValue);
        telemetry.addData("Current RightBack Position", this.robot.right_back_drive.getCurrentPosition());

        switch (state)
        {
            case Move:
                this.moveRobot.StartMove(50, 22.75, 0, -1, 0);
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
