/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;


public class MecanumRotateStateMachine {

    Telemetry telemetry;
    MecanumEncoderMove moveRobot;

    Double degrees;
    MecanumRotateStateMachine.RobotState state;


    enum RobotState
    {
        Start,
        Rotate,
        Rotating,
        Done
    }

    public void init(Telemetry telemetry, MecanumMotor motors) {

        this.telemetry = telemetry;
        this.moveRobot = new MecanumEncoderMove();
        this.moveRobot.init(motors);



        telemetry.addData("Say", "Hello Driver");    //
        state = MecanumRotateStateMachine.RobotState.Start;
    }

    public void Start(Double degrees)
    {
        this.degrees= degrees;
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
                this.moveRobot.StartRotate(telemetry, 70, degrees, MecanumEncoderMove.RotationDirection.Right );
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
