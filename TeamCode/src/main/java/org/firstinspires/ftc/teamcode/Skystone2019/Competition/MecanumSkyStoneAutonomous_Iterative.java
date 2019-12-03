/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.ColorFinder;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.CoreHex;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.GyroController;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.GyroInitStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumSkyStoneStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.SetUpStateMachine;

@Autonomous(name="Mecanum: Skystone", group="Mecanum")
public class MecanumSkyStoneAutonomous_Iterative extends OpMode {

    private HardwareMecanumBase robot;
    private MecanumSkyStoneStateMachine MecanumSkyStoneStateMachine;
    private GyroInitStateMachine gyroInitStateMachine ;
    private MecanumEncoderMove moveRobot;
    private MecanumMotor mecanumRobot;
    private GyroController gyro;
    private ColorFinder colorFinder;
    private CoreHex coreHex;
    private SetUpStateMachine setUpStateMachine;
    private boolean RedTeam;
    private boolean EndByWall;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.robot = new HardwareMecanumBase();
        this.mecanumRobot = new MecanumMotor();
        this.moveRobot = new MecanumEncoderMove();
        this.colorFinder = new ColorFinder();
        this.gyro = new GyroController();
        this.MecanumSkyStoneStateMachine = new MecanumSkyStoneStateMachine();
        this.gyroInitStateMachine = new GyroInitStateMachine();
        this.setUpStateMachine = new SetUpStateMachine();
        this.coreHex = new CoreHex();

        /* Step 2: Setup of hardware  */
        this.robot.init(hardwareMap);
        this.mecanumRobot.init(robot);

        /* Step 3: Setup of controllers  */
        this.moveRobot.init(this.mecanumRobot);
        this.colorFinder.init(hardwareMap);
        this.gyro.init(robot);

        /* Step 4: Setup of state machines  */
        this.gyroInitStateMachine.init(telemetry, this.gyro);
        this.setUpStateMachine.init(telemetry, gamepad1);
        setUpStateMachine.Start();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    @Override
    public void init_loop() {
        this.gyroInitStateMachine.ProcessState();
//        setUpStateMachine.ProcessState();
        telemetry.addData("Gyro Init State", gyroInitStateMachine.IsDone());    //
        telemetry.update();

    }

        /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void start() {
        RedTeam = setUpStateMachine.RedTeam;
        EndByWall = setUpStateMachine.EndWall;

        this.MecanumSkyStoneStateMachine.init(telemetry, mecanumRobot, colorFinder, gyro, EndByWall, RedTeam, robot);
        MecanumSkyStoneStateMachine.Start();

    }

    @Override
    public void loop()
    {
        MecanumSkyStoneStateMachine.ProcessState();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}