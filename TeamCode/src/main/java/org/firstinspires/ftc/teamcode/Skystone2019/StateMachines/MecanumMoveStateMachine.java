/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMove;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumMoveStateMachine {

    Telemetry telemetry;
    HardwareMecanumBase robot = new HardwareMecanumBase(); // use the class created to define a Pushbot's hardware
    MecanumMove moveRobot;
    MecanumMoveStateMachine.RobotState state;


    enum RobotState
    {
        Start,
        Move,
        Moving,
        Done
    }

    public void init(Telemetry telemetry, MecanumMove mecanumMove) {

        this.telemetry = telemetry;
        this.moveRobot = mecanumMove;

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

        switch (state)
        {
            case Move:

                this.moveRobot.StartMove(50, 22.75, 0, 1, 0);
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
