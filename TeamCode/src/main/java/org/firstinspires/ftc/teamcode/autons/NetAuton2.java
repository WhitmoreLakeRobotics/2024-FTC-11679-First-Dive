package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Settings;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Robot;

//@Disabled
@Autonomous(name = "NetAuton2", group = "Auton")
// @Autonomous(...) is the other common choice

public class NetAuton2 extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();




    private stage currentStage = stage._unknown;
    // declare auton power variables
    //private double AUTO_DRIVE_TURBO_SPEED = DriveTrain.DRIVETRAIN_TURBOSPEED;
    //private double AUTO_DRIVE_SLOW_SPEED = DriveTrain.DRIVETRAIN_SLOWSPEED;
    // private double AUTO_DRIVE_NORMAL_SPEED = DriveTrain.DRIVETRAIN_NORMALSPEED;
    // private double AUTO_TURN_SPEED = DriveTrain.DRIVETRAIN_TURNSPEED;

    private String RTAG = "8492-Auton";

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        msStuckDetectInit = Settings.msStuckDetectInit;
        msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        msStuckDetectStart = Settings.msStuckDetectStart;
        msStuckDetectLoop = Settings.msStuckDetectLoop;
        msStuckDetectStop = Settings.msStuckDetectStop;

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        telemetry.addData("Test Auton", "Initialized");

        //Initialize Gyro
        robot.driveTrain.ResetGyro();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // initialize robot
        robot.init_loop();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // start robot
        runtime.reset();
        robot.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.loop();

        switch (currentStage) {
            case _unknown:
                currentStage = stage._00_preStart;
                break;
            case _00_preStart:
                currentStage = stage._40_Left_1;
                break;

            case _40_Left_1:
                robot.driveTrain.CmdDrive(4,0,0.35,0);
                currentStage = stage._60_Left_1_2;
                break;

            case _60_Left_1_2:
                robot.driveTrain.CmdDrive(24,-90,0.35,0);
                currentStage = stage._80_Drive_Right1;
                break;

            case _80_Drive_Right1:
                robot.driveTrain.CmdDrive(51,0,0.35,0);
                currentStage = stage._85_Drive_left1;
                break;

            case _85_Drive_left1:
                robot.driveTrain.CmdDrive(12,90,0.35,0);
                currentStage = stage._90_Reverse1;
                break;

            case _90_Reverse1:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(47,-180,0.35,0);
                    currentStage = stage._100_Forward1;

                }
                break;
            case _100_Forward1:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(47,0,0.35,0);
                    currentStage = stage._105_Drive_left2;
                }

                break;
            case _105_Drive_left2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(8,-90,0.35,0);
                    currentStage = stage._110_Reverse2;
                }

                break;
            case _110_Reverse2:
                if (robot.driveTrain.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(47,-180,0.35,0);
                    currentStage = stage._120_Forward2;
                }

                break;
            case _120_Forward2:
                if (robot.driveTrain.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(47,0,0.35,0);
                    currentStage = stage._125_Drive_left3;
                }

                break;
            case _125_Drive_left3:
                if (robot.driveTrain.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(8,-90,0.35,90);
                    currentStage = stage._130_Reverse3;
                }

                break;
            case _130_Reverse3:
                if (robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(47,-180,0.35,0);
                    currentStage = stage._140_Forward3;
                }

                break;
            case _140_Forward3:
                if (robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(54,0,0.35,0);
                    currentStage = stage._150_HeadR1;
                }

                break;
            case _150_HeadR1:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(0,0,0,90);
                    currentStage = stage._160_Extend_Arm;
                }

                break;
            case _160_Extend_Arm:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(0,0,0,90);
                    robot.arm.setCurrentMode(Arm.Mode.CLIMB);
                    currentStage = stage._170_Drive_Right2;
                    // get from other code;

                }

                break;
            case _170_Drive_Right2:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(28,90,0.35,90);
                    currentStage = stage._180_End;
                    //28 is not actual variable;

                }

                break;
            case _180_End:
                if(robot.driveTrain.getCmdComplete()){
                    robot.stop();


                }

                break;
        }



    }  //  loop

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _40_Left_1,
        _60_Left_1_2,
        _80_Drive_Right1,
        _85_Drive_left1,
        _90_Reverse1,
        _100_Forward1,
        _105_Drive_left2,
        _110_Reverse2,
        _120_Forward2,
        _125_Drive_left3,
        _130_Reverse3,
        _140_Forward3,
        _150_HeadR1,
        _160_Extend_Arm,
        _170_Drive_Right2,
        _180_End
        //contactbar;

    }
}