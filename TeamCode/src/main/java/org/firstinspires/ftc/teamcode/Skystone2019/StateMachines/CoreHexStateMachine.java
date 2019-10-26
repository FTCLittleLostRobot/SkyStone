/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.CoreHex;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.CoreHex.RotationDirection;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;


public class CoreHexStateMachine {

    Telemetry telemetry;
    CoreHex coreHex;
    CoreHexStateMachine.RobotState state;
    RotationDirection rotationDirection;

    enum RobotState
    {
        Start,
        Rotate,
        Rotating,
        Done
    }

    public void init(Telemetry telemetry, HardwareMecanumBase robot, RotationDirection rotationDirection) {

        this.telemetry = telemetry;

        this.coreHex = new CoreHex();
        this.coreHex.init(robot);

        this.rotationDirection = rotationDirection;

        state = CoreHexStateMachine.RobotState.Start;
    }

    public void Start()
    {
        state = CoreHexStateMachine.RobotState.Rotate;
    }

    public boolean IsDone()
    {
        return (state == CoreHexStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        telemetry.addData("Current State", state.toString());

        switch (state)
        {
            case Rotate:
                this.coreHex.Start(telemetry, rotationDirection);
                state = RobotState.Rotating;
                break;

            case Rotating:
                if (this.coreHex.IsDone()) {
                    this.coreHex.Complete();
                    state = RobotState.Done;
                }
                break;


            case Done:
                state = CoreHexStateMachine.RobotState.Done;
                break;
        }
    }
}
