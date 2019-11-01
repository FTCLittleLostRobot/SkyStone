/* Little Lost Robots
   Core Devs: Danielle, Ryan
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import android.graphics.Bitmap;

import com.vuforia.Image;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.ColorFinder;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumSkyStoneStateMachine {

    Telemetry telemetry;
    MecanumEncoderMove moveRobot;
    ColorFinder colorFinder;
    MecanumSkyStoneStateMachine.RobotState state;
    MecanumRotateStateMachine rotateStateMachine;

    // THIS IS IF THE ROBOT IS FACING FORWARDS
    static final double FORWARD_SPEED = 0.1;
    static final double TURN_SPEED = 0.25;
    static final double GO_FORWARD = -1;
    static final double GO_BACK = 1;
    static final double GO_RIGHT = -1;
    static final double GO_LEFT = 1;
    private Image vuforiaImageObject;
    private Bitmap bitmapFromVuforia;
    int foundColumn = -1;
    public int skyStonePosition = -1;


    enum RobotState
    {
        Start,
        MoveForwards,
        MovingForwards,

        CheckScreen,
        ConvertImageFromScreen,
        DetectColorFromImage,
        CheckForSkystone,
        Turn90ToFaceFront,
        Turning90ToFaceFront,
        Turn180ToFaceFront,
        Turning180ToFaceFront,
        MovingTowardsBlock,
        CollectBlock,
        CollectingBlock,
        ReOrientate,
        ReOrienting,
        FaceTowardsSkyBridge,
        FaceingTowardsSkyBridge,

        MoveUnderSkyBridge1,
        MovingUnderSkyBridge1,
        BackUp1,
        BackingUp1,
        SquareAgainstWall,
        SquaringAgainstWall,
        StrafeInfrontOfBlock,
        StrafingInfrontOfBlock,
        MoveTowardsBlock2,
        MovingTowardsBlock2,
        CollectBlock2,
        CollectingBlock2,

        BackingUp2,
        StrafeRight2,
        StrafingRight2,
        DropBlock2,
        DroppingBlock2,
        ParkOnLine,
        ParkingOnLine,
        Done
    }

    public void init(Telemetry telemetry, MecanumMotor motors, ColorFinder colorFinder) {

        this.telemetry = telemetry;
        this.colorFinder = colorFinder;

        this.moveRobot = new MecanumEncoderMove();
        this.moveRobot.init(motors);

        this.rotateStateMachine = new MecanumRotateStateMachine();
        this.rotateStateMachine.init(telemetry,  motors);


        state = MecanumSkyStoneStateMachine.RobotState.Start;
    }

    public void Start()
    {
        rotateStateMachine.Start(90.0);
        state = RobotState.MoveForwards;
    }

    public boolean IsDone()
    {
        return (state == MecanumSkyStoneStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        telemetry.addData("Current State", state.toString());
        telemetry.addData ("Found In", this.foundColumn);
        telemetry.addData("Column 0", colorFinder.column0);
        telemetry.addData("Column 1", colorFinder.column1);
        telemetry.addData("Column 2", colorFinder.column2);

        telemetry.update();
        switch (state)
        {
            case MoveForwards:
                //y = 1 makes it go backwards
                //this.moveRobot.StartMove(50, ConfigFactory.Get().FoundationInchesFromWall, 0, -1, 0);
                this.moveRobot.StartMove(50, 17, 0, MecanumEncoderMove.GO_BACK, 0);
                state = RobotState.MovingForwards;
                break;

            case MovingForwards:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.CheckScreen;
                }
                break;

            case CheckScreen:
                try {
                    vuforiaImageObject = colorFinder.getVuforiaImagefromFrame();
                }
                catch(InterruptedException ex){
                    telemetry.addData("error", ex.getMessage());
                    break;
                }
                state = RobotState.ConvertImageFromScreen;
                break;

            case ConvertImageFromScreen:
                try {
                    bitmapFromVuforia = colorFinder.getBitmapToAnalyze(vuforiaImageObject);
                }
                catch(InterruptedException ex) {
                    telemetry.addData("error", ex.getMessage());
                    break;
                }
                state = RobotState.DetectColorFromImage;
                break;

            case DetectColorFromImage:
                foundColumn =  colorFinder.FindColor(bitmapFromVuforia, ColorFinder.ColorTarget.Black, telemetry);
                telemetry.addData("column", foundColumn);
                state = RobotState.CheckForSkystone;
                break;

            case CheckForSkystone:
                if (foundColumn == 0 )
                {

                    this.moveRobot.StartMove(50, 6, GO_LEFT, GO_FORWARD,0 );
                    state = RobotState.MovingTowardsBlock;
                    skyStonePosition = 0;
                }
                else if (foundColumn == 1 )
                {
                    this.moveRobot.StartMove(50, 6, GO_LEFT, GO_FORWARD,0 );
                    state = RobotState.MovingTowardsBlock;
                    skyStonePosition = 1;

                }
                else if (foundColumn == 2)
                {
                    this.moveRobot.StartMove(50, 6,0, GO_FORWARD,0 );
                    state = RobotState.MovingTowardsBlock;
                    skyStonePosition = 2;

                }

                else if (foundColumn == -1 ) {
                    {
                        state = RobotState.CheckForSkystone;
                    }
                }
                break;

            case MovingTowardsBlock:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.Turn90ToFaceFront;
                }
                break;
            case Turn90ToFaceFront:
                this.moveRobot.StartRotate(telemetry, 70, 180.0, MecanumEncoderMove.RotationDirection.Right );

                state = RobotState.Turning90ToFaceFront;
                break;

            case Turning90ToFaceFront:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.CollectBlock;
                }
                break;



            case CollectBlock:
                this.moveRobot.StartMove(20, 2, 0, GO_FORWARD, 0); // We need the robot to go forwards and turn so the block stays in our grip and doesn't move much.
                break;

            case CollectingBlock:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.ReOrientate;
                }
                break;

            case ReOrientate:
                this.moveRobot.StartMove(20, 4, GO_FORWARD, 0, 0);
                break;

            case ReOrienting:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.FaceingTowardsSkyBridge;
                    rotateStateMachine.Start(90.0);
                }
                break;

            case FaceingTowardsSkyBridge:
                if (this.rotateStateMachine.IsDone()) {
                    state = RobotState.MoveUnderSkyBridge1;
                }
                else
                    rotateStateMachine.ProcessState();

                break;

            case MoveUnderSkyBridge1:
                this.moveRobot.StartMove(50, 42, MecanumEncoderMove.GO_FORWARD, 0, 0);
                state = RobotState.MovingUnderSkyBridge1;
                break;

            case MovingUnderSkyBridge1:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.BackUp1;
                }
                break;

            case BackUp1:
                this.moveRobot.StartMove(50, 47, MecanumEncoderMove.GO_BACK, 0, 0);
                state = RobotState.BackingUp1;
                break;

            case BackingUp1:
                if (this.moveRobot.IsDone()){
                    this.moveRobot.Complete();
                    state = RobotState.SquareAgainstWall;
                }

                break;
            case SquareAgainstWall:
                //todo ADD TIMER FOR PUSHING AGAINST WALL, (pull of Jim FTC person github account)
                this.moveRobot.StartMove(10, 5, GO_BACK, 0, 0);
                state = RobotState.SquaringAgainstWall;
                break;

            case SquaringAgainstWall:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.StrafeInfrontOfBlock;
                }
                break;
            /*
            case BackUp2:
                //y = 1 makes it go backwards
                //this.moveRobot.StartMove(50, ConfigFactory.Get().FoundationInchesFromWall, 0, -1, 0);
                this.moveRobot.StartMove(50, 5, 0, 1, 0);
                state = RobotState.BackingUp2;
                break;

            case BackingUp2:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.StrafeRight2;
                }
                break;

            case StrafeRight2:
                //y = 1 makes it go backwards
                //this.moveRobot.StartMove(50, ConfigFactory.Get().FoundationInchesFromWall, 0, -1, 0);
                this.moveRobot.StartMove(50, 75, 1, 0, 0);
                state = RobotState.StrafingRight2;
                break;

            case StrafingRight2:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.DropBlock2;
                }
                break;

            case Done:
                state = MecanumSkyStoneStateMachine.RobotState.Done;
                break;

             */

        }

    }
}
