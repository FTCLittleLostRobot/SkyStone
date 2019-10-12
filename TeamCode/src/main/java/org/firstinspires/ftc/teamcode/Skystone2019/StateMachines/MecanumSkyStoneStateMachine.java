/* Little Lost Robots
   Core Devs: Danielle, Ryan
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Config.ConfigFactory;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMove;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumSkyStoneStateMachine {

    Telemetry telemetry;
    MecanumMove moveRobot;
    MecanumSkyStoneStateMachine.RobotState state;

    enum RobotState
    {
        Start,
        MoveFowards,
        MovingFowards,
        DetectBlock,
        DetectingBlock,
        MoveTowardsBlock1,
        MovingTowardsBlock1,
        CollectBlock1,
        CollectingBlock1,
        BackUp1,
        BackingUp1,
        StrafeRight1,
        StrafingRight1,
        DropBlock1,
        DroppingBlock1,
        StrafeLeft1,
        StrafingLeft1,
        SquareAgainstWall,
        SquaringAgainstWall,
        StrafeInfrontOfBlock,
        StrafingInfrontOfBlock,
        MoveTowardsBlock2,
        MovingTowardsBlock2,
        CollectBlock2,
        CollectingBlock2,
        BackUp2,
        BackingUp2,
        StrafeRight2,
        StrafingRight2,
        DropBlock2,
        DroppingBlock2,
        ParkOnLine,
        ParkingOnLine,
        Done
    }

    public void init(Telemetry telemetry, HardwareMecanumBase robot) {

        this.telemetry = telemetry;
        this.moveRobot = new MecanumMove();
        this.moveRobot.init(robot);


        state = MecanumSkyStoneStateMachine.RobotState.Start;
    }

    public void Start()
    {
        state = RobotState.MoveFowards;
    }

    public boolean IsDone()
    {
        return (state == MecanumSkyStoneStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        telemetry.addData("Current State", state.toString());

        switch (state)
        {
            case MoveFowards:
                //y = 1 makes it go backwards
                //this.moveRobot.StartMove(50, ConfigFactory.Get().FoundationInchesFromWall, 0, -1, 0);
                this.moveRobot.StartMove(50, 16, 0, -1, 0);
                state = RobotState.MovingFowards;
                break;

            case MovingFowards:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.DetectBlock;
                }
                break;

            case BackUp1:
                //y = 1 makes it go backwards
                //this.moveRobot.StartMove(50, ConfigFactory.Get().FoundationInchesFromWall, 0, -1, 0);
                this.moveRobot.StartMove(50, 5, 0, 1, 0);
                state = RobotState.BackingUp1;
                break;

            case BackingUp1:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.StrafeRight1;
                }
                break;

            case StrafeRight1:
                //y = 1 makes it go backwards
                //this.moveRobot.StartMove(50, ConfigFactory.Get().FoundationInchesFromWall, 0, -1, 0);
                this.moveRobot.StartMove(50, 75, 1, 0, 0);
                state = RobotState.StrafingRight1;
                break;

            case StrafingRight1:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.DropBlock1;
                }
                break;

            case StrafeLeft1:
                //y = 1 makes it go backwards
                //this.moveRobot.StartMove(50, ConfigFactory.Get().FoundationInchesFromWall, 0, -1, 0);
                this.moveRobot.StartMove(50, 75, -1, 0, 0);
                state = RobotState.StrafingLeft1;
                break;

            case StrafingLeft1:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.SquareAgainstWall;
                }
                break;

            case SquareAgainstWall:
                //y = 1 makes it go backwards
                //this.moveRobot.StartMove(50, ConfigFactory.Get().FoundationInchesFromWall, 0, -1, 0);
                this.moveRobot.StartMove(25, 5, -1, 0, 0);
                state = RobotState.SquaringAgainstWall;
                break;

            case SquaringAgainstWall:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.StrafeInfrontOfBlock;
                }
                break;

            case BackUp2:
                //y = 1 makes it go backwards
                //this.moveRobot.StartMove(50, ConfigFactory.Get().FoundationInchesFromWall, 0, -1, 0);
                this.moveRobot.StartMove(50, 5, 0, 1, 0);
                state = RobotState.BackingUp2;
                break;

            case BackingUp2:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.StrafeRight2;
                }
                break;

            case StrafeRight2:
                //y = 1 makes it go backwards
                //this.moveRobot.StartMove(50, ConfigFactory.Get().FoundationInchesFromWall, 0, -1, 0);
                this.moveRobot.StartMove(50, 75, 1, 0, 0);
                state = RobotState.StrafingRight2;
                break;

            case StrafingRight2:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.DropBlock2;
                }
                break;

            case Done:
                state = MecanumSkyStoneStateMachine.RobotState.Done;
                break;
        }
    }
}
