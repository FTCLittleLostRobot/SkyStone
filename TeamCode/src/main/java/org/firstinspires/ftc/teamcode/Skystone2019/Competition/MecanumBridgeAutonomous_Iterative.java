/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.ColorFinder;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.SetUpStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.StayOutOfAlliancesWay;


@Autonomous(name="Mecanum: Autonomous v1", group="Mecanum")
public class MecanumBridgeAutonomous_Iterative extends OpMode {

    private HardwareMecanumBase robot;
    private MecanumEncoderMove moveRobot;
    private MecanumMotor mecanumRobot;
    private StayOutOfAlliancesWay setUpStateMachine;

    boolean BlueTeam;
    boolean RedTeam;
    boolean IsRed;
    boolean StartDepot;
    boolean StartFoundation;
    boolean EndNeutralBridge;
    boolean EndWall;
    boolean JustGoingUnderBridge;

    @Override
    public void init() {
        this.robot = new HardwareMecanumBase();
        this.mecanumRobot = new MecanumMotor();
        this.moveRobot = new MecanumEncoderMove();
        this.robot.init(hardwareMap);
        this.mecanumRobot.init(robot);
        this.moveRobot.init(this.mecanumRobot);
        this.setUpStateMachine = new StayOutOfAlliancesWay(); //setUpStateMachine

        this.setUpStateMachine.init(telemetry, gamepad1);
        setUpStateMachine.Start();

    }

    @Override
    public void init_loop()
    {
        setUpStateMachine.ProcessState();
    }

    @Override
    public void start() {
      /*  RedTeam = setUpStateMachine.RedTeam;
        StartDepot = setUpStateMachine.RedTeam;
        EndWall = setUpStateMachine.EndWall;
        JustGoingUnderBridge = setUpStateMachine.JustGoingUnderBridge;
        StartFoundation = setUpStateMachine.StartFoundation;

       */
        this.moveRobot.StartMove(10, 30, -1, 0, 0);

        /*
        if (RedTeam == true && StartDepot && EndWall && JustGoingUnderBridge) {
            this.moveRobot.StartMove(10, 10, -1, 0, 0);
        }
        else if (RedTeam == true && StartFoundation && EndWall && JustGoingUnderBridge){
            this.moveRobot.StartMove(10, 10, 1, 0, 0);
        }
        else if (RedTeam == true && StartDepot && EndNeutralBridge &&JustGoingUnderBridge){
            this.moveRobot.StartMove(10, 10, 0, -1, 0);
            if (this.moveRobot.IsDone()) {
                this.moveRobot.StartMove(10, 10, -1, 0, 0);
            }
        }
        else if (RedTeam == true && StartFoundation && EndNeutralBridge &&JustGoingUnderBridge){
            this.moveRobot.StartMove(10, 10, 0, -1, 0);
            if (this.moveRobot.IsDone()) {
                this.moveRobot.StartMove(10, 10, 1, 0, 0);
            }
        }

        if (BlueTeam = true && StartDepot && EndWall && JustGoingUnderBridge) {
            this.moveRobot.StartMove(10, 10, 1, 0, 0);
        }
        else if (BlueTeam = true && StartFoundation && EndWall && JustGoingUnderBridge){
            this.moveRobot.StartMove(10, 10, -1, 0, 0);
        }

        else if (BlueTeam == true && StartDepot && EndNeutralBridge &&JustGoingUnderBridge){
            this.moveRobot.StartMove(10, 10, 0, -1, 0);
            if (this.moveRobot.IsDone()) {
                this.moveRobot.StartMove(10, 10, 1, 0, 0);
            }
        }

        else if (BlueTeam == true && StartFoundation && EndNeutralBridge &&JustGoingUnderBridge){
            this.moveRobot.StartMove(10, 10, 0, -1, 0);
            if (this.moveRobot.IsDone()) {
                this.moveRobot.StartMove(10, 10, -1, 0, 0);
            }
        }
  */
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        if (this.moveRobot.IsDone()) {
            this.moveRobot.Complete();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}