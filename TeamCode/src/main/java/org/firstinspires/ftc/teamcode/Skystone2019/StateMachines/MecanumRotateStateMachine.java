/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMove;


public class MecanumRotateStateMachine {

    Telemetry telemetry;
    MecanumMove moveRobot;

    Double degrees;
    MecanumRotateStateMachine.RobotState state;


    enum RobotState
    {
        Start,
        Rotate,
        Rotating,
        Done
    }

    public void init(Telemetry telemetry, Double degrees , HardwareMecanumBase robot) {

        this.telemetry = telemetry;
        this.moveRobot = new MecanumMove();
        this.degrees= degrees;
        this.moveRobot.init(robot);



        telemetry.addData("Say", "Hello Driver");    //
        state = MecanumRotateStateMachine.RobotState.Start;
    }

    public void Start()
    {
        state = MecanumRotateStateMachine.RobotState.Rotate;
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
            case Rotate:
                this.moveRobot.StartRotate(telemetry, 70, degrees, MecanumMove.RotationDirection.Right );
                state = RobotState.Rotating;
                break;

            case Rotating:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.Done;
                }
                break;


            case Done:
                state = MecanumRotateStateMachine.RobotState.Done;
                break;
        }
    }
}
