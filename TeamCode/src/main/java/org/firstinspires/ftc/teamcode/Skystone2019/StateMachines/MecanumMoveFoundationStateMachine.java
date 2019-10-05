/* Little Lost Robots
   Core Devs: Danielle, Ryan
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.robot.RobotState;
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
        UnderBridge,
        MovingUnderBridge,
        UnderBridge2,
        MovingUnderBridge2,
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
                state = RobotState.MovingStraight;
                break;

            case MovingStraight:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.PullBack;
                }
                break;

            case PullBack:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, 35, 0, 1, 0);
                state = RobotState.PullingBack;
                break;

            case PullingBack:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.StrafeLeft;
                }
                break;

            case StrafeLeft:
                //x = -1 makes it go right
                this.moveRobot.StartMove(50, 30, 1, 0, 0);
                state = RobotState.StrafingLeft;
                break;

            case StrafingLeft:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.MoveFowards;
                }
                break;

            case MoveFowards:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, 16, 0, -1, 0);
                state = RobotState.MovingFowards;
                break;

            case MovingFowards:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.PushPlatform;
                }
                break;

            case PushPlatform:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, 8, -1, 0, 0);
                state = RobotState.PushingPlatform;
                break;

            case PushingPlatform:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.UnderBridge;
                }
                break;

            case UnderBridge:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, 26, 1, 1, 0);
                state = RobotState.MovingUnderBridge;
                break;

            case MovingUnderBridge:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.UnderBridge2
                    ;
                }
                break;

            case UnderBridge2:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, 18, 1, 0, 0);
                state = RobotState.MovingUnderBridge2;
                break;

            case MovingUnderBridge2:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.Done
                    ;
                }
                break;



            case Done:
                state = MecanumMoveFoundationStateMachine.RobotState.Done;
                break;
        }
    }
}
