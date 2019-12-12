/* Little Lost Robots
   Core Devs: Danielle
*/
/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Config.IConfiguration;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.ColorFinder;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.CoreHex;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.GyroController;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumStayOutOfAllianceWayStateMachine {

    Telemetry telemetry;
    MecanumEncoderMove moveRobot;
    ColorFinder colorFinder;
    MecanumStayOutOfAllianceWayStateMachine.RobotState state;
    SetUpStateMachine setUpStateMachine;
    IConfiguration robotConfig;
    MecanumMotor motors;
    private GyroInitStateMachine gyroInitStateMachine;
    private MecanumRotateStateMachine rotateStateMachine;

    // THIS IS IF THE ROBOT IS FACING FORWARDS
    static final double GO_FORWARD = -1;
    static final double GO_BACK = 1;
    ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum RobotState
    {
        Start,
        MoveForwards,
        MovingForwards,

        Done
    }

    public void init(Telemetry telemetry, MecanumMotor motors, ColorFinder colorFinder, GyroController gyro, boolean EndByWall, boolean isRed, HardwareMecanumBase robot ) {

        this.telemetry = telemetry;
        this.colorFinder = colorFinder;

        this.moveRobot = new MecanumEncoderMove();
        this.moveRobot.init(motors);
        this.motors = motors;
        state = MecanumStayOutOfAllianceWayStateMachine.RobotState.Start;
    }

    public void Start()
    {
        state = RobotState.MoveForwards;
    }

    private void CheckIfDone(RobotState nextState) {
        if (this.moveRobot.IsDone()) {
            this.moveRobot.Complete();
            state = nextState;
        }
    }
    public boolean IsDone()
    {
        return (state == MecanumStayOutOfAllianceWayStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        telemetry.addData("Current State", state.toString());

        telemetry.addData("targetLeftFrontEncoderValue", moveRobot.targetLeftFrontEncoderValue);
        telemetry.addData("CurrentLeftFrontPosition", moveRobot.GetLeftMotorEncodePosition());
        telemetry.update();


        switch (state)
        {
            case MoveForwards:
                this.moveRobot.StartMove(30, 27, 0, MecanumEncoderMove.GO_BACK, 0);
                state = RobotState.MovingForwards;
                break;

            case MovingForwards:
                this.CheckIfDone(RobotState.Done);
                break;


        }

    }
}
