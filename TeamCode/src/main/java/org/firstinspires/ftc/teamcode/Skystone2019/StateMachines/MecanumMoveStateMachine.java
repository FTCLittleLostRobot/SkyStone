/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.robot.RobotState;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMove;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumMoveStateMachine {

    Telemetry telemetry;
    MecanumMove moveRobot;
    MecanumMoveStateMachine.RobotState state;




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

        switch (state)
        {
            case Move:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, 22.75, 1, 0, 0);
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
