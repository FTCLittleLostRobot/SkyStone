/*
   Little Lost Robots
   Core Devs: Danielle
*/


package org.firstinspires.ftc.teamcode.Skystone2019.Competition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.CoreHex;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.CoreHexStateMachine;
/*
 *   This program should be used to make sure all wires are in and in the correct ports.
 *   If a wire is in the wrong port or not in at all this program will let you know!
 */

@TeleOp(name="Mecanum: Setup", group="TestingMecanum")

public class MecanumSetup_Teleop extends OpMode {

    /* Declare OpMode members. */
    private HardwareMecanumBase robot = new HardwareMecanumBase(); // use the class created to define a Mencanums 's hardware
    private MecanumMotor mecanumMotor = new MecanumMotor();
    private CoreHexStateMachine coreHexStateMachineLifter = new CoreHexStateMachine();
    private CoreHexStateMachine coreHexStateMachineGrabber = new CoreHexStateMachine();

    private DcMotor left_front_drive = null;   //front left wheel
    private DcMotor right_front_drive = null;  //front right wheel
    private DcMotor left_back_drive = null;    //back left wheel
    private DcMotor right_back_drive = null;   //back right wheel

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        mecanumMotor.init(robot);
        coreHexStateMachineLifter.init(telemetry, robot, CoreHex.CoreHexMotors.BlockLifter);
        coreHexStateMachineGrabber.init(telemetry, robot, CoreHex.CoreHexMotors.BlockGrabber);


        // Is every motor on correctly? This checks each motor and lift.
        this.CheckMotor(robot.left_front_drive, "left_front");
        this.CheckMotor(robot.right_front_drive, "right_front");
        this.CheckMotor(robot.left_back_drive, "left_back");
        this.CheckMotor(robot.right_back_drive, "right_back");
        this.CheckMotor(robot.Block_Grabber, "Block_Grabber");
        this.CheckMotor(robot.Block_Lifter, "Block_Lifter");

        telemetry.addData("Say", "Hello Driver");    //this shows the robot is ready

    }

    private void CheckMotor(DcMotor motor, String  motorName)
    {
        // if a motor is not found it will tell the driver which wheel isn't in
        // this helps drivers tell why the program is giving an error and how to fix it easily
        if (motor == null) {
            telemetry.addData("Error", motorName + " not found");
        }
    }

    @Override
    public void loop () {

        // this moves the right front wheel; You need to press the left up
        if (gamepad1.left_stick_y == -1) {
            mecanumMotor.DrivePower(MecanumMotor.WheelControl.RightFrontDrive, -0.3);
            telemetry.addData("Testing:", "right_front");
        }
        else{
            mecanumMotor.DrivePower(MecanumMotor.WheelControl.RightFrontDrive, 0);
        }

        // this moves the left front wheel; You need to press the left stick to the left
        if (gamepad1.left_stick_x == -1) {
            mecanumMotor.DrivePower(MecanumMotor.WheelControl.LeftFrontDrive, -0.3);
            telemetry.addData("Testing:", "left_front");
        }
        else{
            mecanumMotor.DrivePower(MecanumMotor.WheelControl.LeftFrontDrive, 0);
        }

        // this moves the left back wheel; You need to press the left stick down
        if (gamepad1.left_stick_y == 1) {
            mecanumMotor.DrivePower(MecanumMotor.WheelControl.LeftBackDrive, -0.3);
            telemetry.addData("Testing:", "left_back");
        }
        else{
            mecanumMotor.DrivePower(MecanumMotor.WheelControl.LeftBackDrive, 0);
        }

        // this moves the right back wheel; You need to press the left stick to the right
        if (gamepad1.left_stick_x == 1) {
            mecanumMotor.DrivePower(MecanumMotor.WheelControl.RightBackDrive, -0.3);
            telemetry.addData("Testing:", "right_back");
        }
        else{
            mecanumMotor.DrivePower(MecanumMotor.WheelControl.RightBackDrive, 0);
        }

        if (gamepad2.x){
            coreHexStateMachineGrabber.Start(CoreHex.RotationDirection.Up);
            coreHexStateMachineGrabber.ProcessState();
        }
        else if ( gamepad2.b){
            coreHexStateMachineGrabber.Start(CoreHex.RotationDirection.Down);
            coreHexStateMachineGrabber.ProcessState();
        }

        if (gamepad2.y){
            coreHexStateMachineLifter.Start(CoreHex.RotationDirection.Up);
            coreHexStateMachineGrabber.ProcessState();
        }
        else if (gamepad2.a){
            coreHexStateMachineLifter.Start(CoreHex.RotationDirection.Down);
            coreHexStateMachineGrabber.ProcessState();
        }
        coreHexStateMachineLifter.ProcessState();

    }
}

