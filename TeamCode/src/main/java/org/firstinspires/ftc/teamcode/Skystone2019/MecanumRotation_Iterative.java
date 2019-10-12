/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumRotateStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMove;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumMoveStateMachine;

@Autonomous(name="Mecanum:Rotation", group="Mecanum")
public class MecanumRotation_Iterative extends OpMode {

    private HardwareMecanumBase robot;
    private MecanumRotateStateMachine rotateStateMachine;
    private MecanumMoveStateMachine moveStateMachine;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.robot = new HardwareMecanumBase();
        this.rotateStateMachine = new MecanumRotateStateMachine();
        this.moveStateMachine = new MecanumMoveStateMachine();

        /* Step 2: Setup of hardware  */
        robot.init(hardwareMap);

        /* Step 3: Setup of controllers  */

        /* Step 4: Setup of state machines  */
        this.rotateStateMachine.init(telemetry, 180.0 , robot);
        this.moveStateMachine.init(telemetry, robot);

//        this.rotateStateMachine = new MecanumRotateStateMachine();
  //      this.rotateStateMachine.init(telemetry, 90.0 , robot);

        //moveRobot.StartRotate(telemetry, 2, 90, MecanumMove.RotationDirection.Right);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    @Override
    public void start() {
        rotateStateMachine.Start();
        moveStateMachine.Start();
    }

    @Override
    public void loop()
    {
        moveStateMachine.ProcessState();

//        if (moveStateMachine.IsDone()) {
//            rotateStateMachine.ProcessState();
//        }
    }

    @Override
    public void stop() {
    }
}