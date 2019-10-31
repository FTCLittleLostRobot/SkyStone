/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.SetUpStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumMoveStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumRotateStateMachine;

@Autonomous(name="Mecanum:Test (random code)", group="Mecanum")
public class MecanumTesting_Iterative extends OpMode {

    private HardwareMecanumBase robot;
    private MecanumRotateStateMachine rotateStateMachine;
    private MecanumMoveStateMachine moveStateMachine;
    private SetUpStateMachine setUpStateMachine;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.robot = new HardwareMecanumBase();
        this.rotateStateMachine = new MecanumRotateStateMachine();
        this.moveStateMachine = new MecanumMoveStateMachine();
        this.setUpStateMachine = new SetUpStateMachine();

        /* Step 2: Setup of hardware  */
        robot.init(hardwareMap);

        /* Step 3: Setup of controllers  */

        /* Step 4: Setup of state machines  */
        this.rotateStateMachine.init(telemetry, robot);
        this.moveStateMachine.init(telemetry, robot);
        this.setUpStateMachine.init(telemetry, gamepad1);

//        this.rotateStateMachine = new MecanumRotateStateMachine();
  //      this.rotateStateMachine.init(telemetry, 90.0 , robot);

        //moveRobot.StartRotate(telemetry, 2, 90, MecanumMove.RotationDirection.Right);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    @Override
    public void start() {
        rotateStateMachine.Start(90.0);
        moveStateMachine.Start();
        setUpStateMachine.Start();
    }

    @Override
    public void loop()
    {
        setUpStateMachine.ProcessState();

//        moveStateMachine.ProcessState();

//        if (moveStateMachine.IsDone()) {
//            rotateStateMachine.ProcessState();
//        }
    }

    @Override
    public void stop() {
    }
}