/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.SetUpStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumMoveStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumRotateStateMachine;

@Autonomous(name="Mecanum:Test (random code)", group="Mecanum")
public class MecanumTesting_Iterative extends OpMode {

    private HardwareMecanumBase robot = new HardwareMecanumBase();
    private MecanumMotor motors = new MecanumMotor();
    private MecanumRotateStateMachine rotateStateMachine;
    private MecanumMoveStateMachine moveStateMachine;
    private SetUpStateMachine setUpStateMachine;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.rotateStateMachine = new MecanumRotateStateMachine();
        this.moveStateMachine = new MecanumMoveStateMachine();
        this.setUpStateMachine = new SetUpStateMachine();

        /* Step 2: Setup of hardware  */
        robot.init(hardwareMap);
        motors.init(robot);

        /* Step 3: Setup of controllers  */

        /* Step 4: Setup of state machines  */
        this.rotateStateMachine.init(telemetry, motors);
        this.moveStateMachine.init(telemetry, motors);
        this.setUpStateMachine.init(telemetry, gamepad1);

//        this.rotateStateMachine = new MecanumRotateStateMachine();
  //      this.rotateStateMachine.init(telemetry, 90.0 , robot);

        //moveRobot.StartRotate(telemetry, 2, 90, MecanumEncoderMove.RotationDirection.Right);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        setUpStateMachine.Start();
    }

    @Override
    public void init_loop()
    {
        setUpStateMachine.ProcessState();
    }

    @Override
    public void start() {


//        rotateStateMachine.Start(90.0);
//        moveStateMachine.Start(10, 0, 1, 0)
    }

    @Override
    public void loop()
    {
      //  setUpStateMachine.ProcessState();

//        moveStateMachine.ProcessState();

//        if (moveStateMachine.IsDone()) {
           rotateStateMachine.ProcessState();
//        }
    }

    @Override
    public void stop() {
    }
}