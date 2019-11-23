/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.GyroController;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.GyroInitStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumRotateStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumMoveStateMachine;

@Autonomous(name="Mecanum:RotateTest", group="Mecanum")
public class MecanumRotateTest_Iterative extends OpMode {

    private HardwareMecanumBase robot = new HardwareMecanumBase();
    private MecanumMotor motors = new MecanumMotor();
    private GyroController gyro = new GyroController();
    private GyroInitStateMachine gyroInitStateMachine;
    private MecanumRotateStateMachine rotateStateMachine;
    private MecanumMoveStateMachine moveStateMachine;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.gyroInitStateMachine = new GyroInitStateMachine();
        this.rotateStateMachine = new MecanumRotateStateMachine();
        this.moveStateMachine = new MecanumMoveStateMachine();
        this.gyro = new GyroController();

        /* Step 2: Setup of hardware  */
        robot.init(hardwareMap);
        motors.init(robot);

        /* Step 3: Setup of controllers  */
        this.gyro.init(robot);

        /* Step 4: Setup of state machines  */
        //this.rotateStateMachine.init(telemetry, motors);
        this.moveStateMachine.init(telemetry, motors);
        this.rotateStateMachine.init(telemetry, motors, gyro);
        this.gyroInitStateMachine.init(telemetry, gyro);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        gyroInitStateMachine.ProcessState();
        telemetry.addData("Gyro Init State", gyroInitStateMachine.IsDone());    //
        telemetry.update();
    }

    @Override
    public void start() {
        rotateStateMachine.StartWithGyro(90.0, 20);
        //moveStateMachine.Start(2, 1, -1, 0);
    }

    @Override
    public void loop()
    {
//        moveStateMachine.ProcessState();
          rotateStateMachine.ProcessState();
        telemetry.addData("Gyro heading", gyro.GetCurrentRobotHeading());    //
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}