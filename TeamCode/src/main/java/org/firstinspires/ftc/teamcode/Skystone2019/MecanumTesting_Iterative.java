/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.GyroController;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.GyroInitStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.SetUpStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumMoveStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumRotateStateMachine;

@Autonomous(name="Mecanum:Test (random code)", group="Mecanum")
public class MecanumTesting_Iterative extends OpMode {

    private HardwareMecanumBase robot = new HardwareMecanumBase();
    private MecanumMotor motors = new MecanumMotor();
    private GyroController gyro = new GyroController();
    private MecanumRotateStateMachine rotateStateMachine;
    private MecanumMoveStateMachine moveStateMachine;
    private SetUpStateMachine setUpStateMachine;
    private GyroInitStateMachine gyroInitStateMachine;
    private MecanumRotateStateMachine mecanumRotateStateMachine;
    MecanumEncoderMove moveRobot;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.rotateStateMachine = new MecanumRotateStateMachine();
        this.moveStateMachine = new MecanumMoveStateMachine();
        //   this.setUpStateMachine = new SetUpStateMachine();
        this.moveRobot = new MecanumEncoderMove();
        this.moveRobot.init(motors);
        //  this.mecanumRotateStateMachine = new MecanumRotateStateMachine();
        // this.gyro = new GyroController();

        /* Step 2: Setup of hardware  */
        robot.init(hardwareMap);
        motors.init(robot);

        /* Step 3: Setup of controllers  */
        // this.gyro.init(robot);

        /* Step 4: Setup of state machines  */
        this.moveStateMachine.init(telemetry, motors);
        // this.setUpStateMachine.init(telemetry, gamepad1);
        // this.gyroInitStateMachine.init(telemetry, this.gyro);
        // this.mecanumRotateStateMachine.init(telemetry, motors, this.gyro);

        //     moveRobot.StartRotate(telemetry, 2, 90, MecanumEncoderMove.RotationDirection.Right);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        //   setUpStateMachine.Start();
        //mecanumGyroRotateStateMachine.Start(20, 180);
    }

    @Override
    public void init_loop()
    {
        //gyroInitStateMachine.ProcessState();
        //setUpStateMachine.ProcessState();
        // mecanumGyroRotateStateMachine.ProcessState();
    }

    @Override
    public void start() {


        //       rotateStateMachine.Start(90.0);
       // moveStateMachine.Start(20, -1, 0, 0);
        this.moveRobot.StartMove(40, 12, 0 , -1,0 );

    }

    @Override
    public void loop()
    {
        //  setUpStateMachine.ProcessState();
        //       mecanumGyroRotateStateMachine.ProcessState();
        moveStateMachine.ProcessState();

//        if (moveStateMachine.IsDone()) {
        // rotateStateMachine.ProcessState();
//        }
    }

    @Override
    public void stop() {
    }
}