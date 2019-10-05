/* Little Lost Robots
   Core Devs: Danielle, Ryan
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMove;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumMoveFoundationStateMachine {

    Telemetry telemetry;
    MecanumMove moveRobot;
    MecanumMoveFoundationStateMachine.RobotState state;




    enum RobotState
    {
        Start,
        MoveStraight,
        MovingStraight,
        PullBack,
        PullingBack,
        StrafeLeft,
        StrafingLeft,
        MoveFowards,
        MovingFowards,
        PushPlatform,
        PushingPlatform,
        Done
    }

    public void init(Telemetry telemetry, HardwareMecanumBase robot) {

        this.telemetry = telemetry;
        this.moveRobot = new MecanumMove();
        this.moveRobot.init(robot);


        state = MecanumMoveFoundationStateMachine.RobotState.Start;
    }

    public void Start()
    {
        state = RobotState.MoveStraight;
    }

    public boolean IsDone()
    {
        return (state == MecanumMoveFoundationStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        telemetry.addData("Current State", state.toString());

        switch (state)
        {
            case MoveStraight:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, 35, 0, -1, 0);
                state = RobotState.MovingFowards;
                break;

            case MovingFowards:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.Done;
                }
                break;







            case Done:
                state = MecanumMoveFoundationStateMachine.RobotState.Done;
                break;
        }
    }
}
