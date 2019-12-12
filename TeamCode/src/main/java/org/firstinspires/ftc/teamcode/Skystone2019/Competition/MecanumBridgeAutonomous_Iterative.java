/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.StayOutOfAlliancesWayCollectInfo;


@Autonomous(name="Mecanum: Autonomous v1", group="Mecanum")
public class MecanumBridgeAutonomous_Iterative extends OpMode {

    private HardwareMecanumBase robot;
    private MecanumEncoderMove moveRobot;
    private MecanumMotor mecanumRobot;
    private StayOutOfAlliancesWayCollectInfo stayOutOfAlliancesWay;


    boolean MoveForwards;
    boolean MoveBackwards;
    boolean ParkNeutralBridge;
    int WaitTimeAutonomous;


    @Override
    public void init() {
        this.robot = new HardwareMecanumBase();
        this.mecanumRobot = new MecanumMotor();
        this.moveRobot = new MecanumEncoderMove();
        this.robot.init(hardwareMap);
        this.mecanumRobot.init(robot);
        this.moveRobot.init(this.mecanumRobot);

        this.stayOutOfAlliancesWay = new StayOutOfAlliancesWayCollectInfo();
        this.stayOutOfAlliancesWay.init(telemetry, gamepad1);
        stayOutOfAlliancesWay.Start();

    }

    @Override
    public void init_loop()
    {
        stayOutOfAlliancesWay.ProcessState();
    }

    @Override
    public void start() {
        MoveForwards = stayOutOfAlliancesWay.MoveForwards;
        MoveBackwards = stayOutOfAlliancesWay.MoveBackwards;
        ParkNeutralBridge = stayOutOfAlliancesWay.ParkNeutralBridge;
        WaitTimeAutonomous = stayOutOfAlliancesWay.WaitTimeAutonomous;

        if (MoveForwards == true){
            this.moveRobot.StartMove(30, 15, 0, MecanumEncoderMove.GO_FORWARD, 0); //todo: check where to line up
        }else if (MoveBackwards == true){
            this.moveRobot.StartMove(30, 15, 0, MecanumEncoderMove.GO_BACK, 0); //todo: check where to line up
        }

        if (ParkNeutralBridge == true){

        }

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