package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Settings;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Robot;


//@Disabled
@Autonomous(name = "NetAuton7", group = "Auton")
// @Autonomous(...) is the other common choice

public class NetAuton7 extends OpMode {

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
        robot.arm.resetEncoders();
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
                robot.thePointyStick.setNoTouchy();
                break;
            case _00_preStart:
                currentStage = stage._10_Forward0;
                break;
            case _10_Forward0:
                robot.driveTrain.CmdDrive(3, 0, 0.35, 0);
                currentStage = stage._20_TurnL;
                break;
            case _20_TurnL:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdTurn(-90,0.35);
                    robot.arm.setCurrentMode(Arm.Mode.INTERMEDIATE);
                    currentStage = stage._30_Deliver_To_High_Basket;



                }
                break;
            case _30_Deliver_To_High_Basket:
                if (robot.arm.getCmdComplete())     {
                    // robot.driveTrain.CmdDrive(0,-90,0,-90);
                    robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_BASKET);
                    currentStage = stage._35_Reverse0;
                    // get from other code;
                }
                break;



            case _35_Reverse0:
                if (robot.arm.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(2,-180,0.35, -90);
                    currentStage = stage._40_BearL1;
                }
                break;

            case _40_BearL1:
                if (robot.driveTrain.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(10,-120,0.45, -90);
                    currentStage = stage._48_Pause;
                }
                break;

            case _48_Pause:
                if (robot.driveTrain.getCmdComplete()){
                    runtime.reset();
                    currentStage = stage._50_OutPut;
                }
                break;
            case _50_OutPut:
                if (runtime.milliseconds() >= 400){
                    //robot.driveTrain.CmdDrive(0,-90,0,-90);
                    robot.intake.doOut();
                    runtime.reset();
                    currentStage = stage._60_BearR1;
                    // put timer above;
                }

                break;
            case _60_BearR1:
                if (runtime.milliseconds() >= 300) {
                    robot.driveTrain.CmdDrive(8.5,90,0.35,-90);
                    currentStage = stage._70_Arm_Retract;
                }

                break;
            case _70_Arm_Retract:
                if(robot.driveTrain.getCmdComplete()){
                    //robot.driveTrain.CmdDrive(0,0,0,-90);
                    robot.arm.setCurrentMode(Arm.Mode.INTERMEDIATE);
                    robot.intake.doStop();
                    currentStage = stage._80_Forward1;
                    // get from other code;
                }
                 break;
            case _80_Forward1:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(4, 0, 0.35, -90);
                    currentStage = stage._82_Forward1_2;
                }
                break;

            case _82_Forward1_2:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(31, 0, 0.60, -90); // was 29;
                    robot.arm.setCurrentMode(Arm.Mode.PICKUP_GROUND);
                    currentStage = stage._84_Forward1_3;
                }
                break;

            case _84_Forward1_3:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdDriveBySensors(40, 0, 0.35, -90, DriveTrain.SensorSel.LEFT_SIDE);
                    robot.intake.doIn();
                    currentStage = stage._85_Arm_Pickup_Tank1;
                }
                break;

            case _85_Arm_Pickup_Tank1:
                if(robot.driveTrain.getCmdComplete()) {
                    //robot.driveTrain.CmdDrive(0, 0, 0.35, -90);
                  //  robot.arm.setCurrentMode(Arm.Mode.PICKUP_TANK);
                    currentStage = stage._87_Snuffle;
                }
                break;

            case _87_Snuffle:
                if(robot.arm.getCmdComplete()) {
                    //robot.driveTrain.CmdDrive(0, 0, 0.35, -90);
                    runtime.reset();
                    currentStage = stage._90_Snuffle2;
                }
                break;

            case _90_Snuffle2:
                if(robot.intake.BTargetFound || (runtime.milliseconds() >= 900)){
                    //robot.driveTrain.CmdDrive(24,-145,0.35,-135);
                    robot.arm.setCurrentMode(Arm.Mode.INTERMEDIATE);
                    currentStage = stage._95_Turn1;

                }

                else{
                    robot.arm.updateExtension(1.00);
                }
                break;
            case _95_Turn1:
             if(robot.arm.getCmdComplete()){
                 robot.driveTrain.cmdTurn(-135,0.35);
                 currentStage = stage._100_Delivery1;
             }


                break;
            case _100_Delivery1:
                if (robot.driveTrain.getCmdComplete())     {
                    //robot.driveTrain.CmdDrive(0,-180,0.35,-90);
                    robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_BASKET);
                    currentStage = stage._110_Scoring_Drive1;
                }

                break;
            case _110_Scoring_Drive1:
                if (robot.driveTrain.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(4,-145,0.35,-135);
                    runtime.reset();
                    currentStage = stage._112_Scoring_Drive1_2;
                }

                break;

            case _112_Scoring_Drive1_2:
                if (robot.driveTrain.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(19,-145,0.60,-135);
                    runtime.reset();
                    currentStage = stage._114_Scoring_Drive1_3;
                }

                break;

            case _114_Scoring_Drive1_3:
                if (robot.driveTrain.getCmdComplete() && robot.arm.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(4,-145,0.35,-135);
                    runtime.reset();
                    currentStage = stage._117_Pause2;
                }

                break;

            case _117_Pause2:
                if (robot.driveTrain.getCmdComplete()){
                    runtime.reset();
                    currentStage = stage._120_Output2;
                }
                break;

            case _120_Output2:
                if (runtime.milliseconds() >= 400)  {
                    //500 was 1000 above;
                    //robot.driveTrain.CmdDrive(8,90,0.35,-90);
                    robot.intake.doOut();
                    runtime.reset();
                    currentStage = stage._125_Drive_Out1;
                }
                //500 was 1000 above;

            break;
            case _125_Drive_Out1:
                if (runtime.milliseconds() >= 300)  {
                    robot.driveTrain.CmdDrive(4,15,0.35,-135);
                    currentStage = stage._130_Arm_Inter;
                }

                break;
            case _130_Arm_Inter:
                if (robot.driveTrain.getCmdComplete()){
                    // robot.driveTrain.CmdDrive(47,-180,0.35,-90);
                    robot.arm.setCurrentMode(Arm.Mode.INTERMEDIATE);
                    currentStage = stage._140_Drive_Out1_2;
                }
                break;
            case _140_Drive_Out1_2:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(14, 15, 0.60, -135);
                    currentStage = stage._145_Turn1_1;
                    //change 18;
                }
                break;

            case _145_Turn1_1:
                if(robot.driveTrain.getCmdComplete() && robot.arm.getCmdComplete()) {
                    robot.arm.setCurrentMode(Arm.Mode.PICKUP_GROUND);
                    robot.driveTrain.cmdTurn(-90,0.35);
                    currentStage = stage._150_Drive_Out1_3;
                }


                break;

            case _150_Drive_Out1_3:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdDriveBySensors(39.5, 15, 0.35, -90, DriveTrain.SensorSel.LEFT_SIDE); //was 4 distance;
                    currentStage = stage._160_Forward2_3;
                }
                break;

            case _160_Forward2_3:
                if(robot.driveTrain.getCmdComplete()) {
                   // robot.driveTrain.CmdDrive(3, -90, 0.35, -87);

                    robot.intake.doIn();
                    currentStage = stage._170_Arm_Pickup_Tank2;
                }
                break;

            case _170_Arm_Pickup_Tank2:
                if(robot.driveTrain.getCmdComplete()) {
                    //robot.driveTrain.CmdDrive(0, 0, 0.35, -90);
                    // robot.arm.setCurrentMode(Arm.Mode.PICKUP_GROUND);
                    currentStage = stage._180_Snuffle3;
                }
                break;

            case _180_Snuffle3:
                if(robot.arm.getCmdComplete()) {
                    //robot.driveTrain.CmdDrive(0, 0, 0.35, -90);
                    runtime.reset();
                    currentStage = stage._190_Snuffle4;
                }
                break;

            case _190_Snuffle4:
                if(robot.intake.BTargetFound || (runtime.milliseconds() >= 900)){
                    //robot.driveTrain.CmdDrive(24,-180,0.35,-90);
                    robot.arm.setCurrentMode(Arm.Mode.INTERMEDIATE);
                    currentStage = stage._195_Turn2;

                }

                else{
                    robot.arm.updateExtension(1.00);
                }

                break;
            case _195_Turn2:
                if(robot.arm.getCmdComplete()){
                    robot.driveTrain.cmdTurn(-142,0.35);
                    currentStage = stage._200_Delivery2;
                }

                break;
            case _200_Delivery2:
                if (robot.driveTrain.getCmdComplete())     {
                    //robot.driveTrain.CmdDrive(0,-180,0.35,-90);
                    //robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_BASKET);
                    currentStage = stage._210_Scoring_Drive2;
                }

                break;
            case _210_Scoring_Drive2:
                if (robot.arm.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(4,-160,0.35,-142);
                    robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_BASKET);
                    currentStage = stage._220_Scoring_Drive2_2;
                }

                break;
            case _220_Scoring_Drive2_2:
                if (robot.driveTrain.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(16,-160,0.60,-142);
                   // robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_BASKET);
                    currentStage = stage._225_Scoring_Drive1_3;
                }
                  break;
            case _225_Scoring_Drive1_3:
                if (robot.driveTrain.getCmdComplete() && robot.arm.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(3,-160,0.35,-142);
                    runtime.reset();
                    currentStage = stage._227_Pause3;
                }
               break;
                       //all 150s were 160s;

            case _227_Pause3:
                if (robot.driveTrain.getCmdComplete()){
                    runtime.reset();
                    currentStage = stage._230_Output3;
                }


                break;
            case _230_Output3:
                if (runtime.milliseconds() >= 400)  {
                    //500 was 1000;
                    //robot.driveTrain.CmdDrive(8,90,0.35,-90);
                    robot.intake.doOut();
                    runtime.reset();
                    currentStage = stage._240_Drive_Out3;
                }

                break;
            case _240_Drive_Out3:
                if (runtime.milliseconds() >= 400)  {
                    robot.driveTrain.CmdDrive(4,20,0.35,-142);
                    currentStage = stage._250_Arm_Inter2;
                }

                break;
            case _250_Arm_Inter2:
                if (robot.driveTrain.getCmdComplete()){
                    // robot.driveTrain.CmdDrive(47,-180,0.35,-90);
                    robot.arm.setCurrentMode(Arm.Mode.INTERMEDIATE);
                    currentStage = stage._260_Drive_Out3_2;
                }

                break;
            case _260_Drive_Out3_2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(19,20,0.60,-90);
                    currentStage = stage._265_Turn2_2;
                }
                 break;
            case _265_Turn2_2:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.intake.doIn();
                    robot.arm.setCurrentMode(Arm.Mode.PICKUP_GROUND);
                    robot.driveTrain.cmdTurn(-90,0.35);
                    currentStage = stage._270_Drive_Out3_3;
                }

                break;
            case _270_Drive_Out3_3:
                if (robot.driveTrain.getCmdComplete())     {
                    //robot.driveTrain.cmdDriveBySensors(39.5,20,0.35,-90);
                    //robot.arm.setCurrentMode(Arm.Mode.PICKUP_TANK);
                    currentStage = stage._280_Drive_left8;
                }

                break;
            case _280_Drive_left8:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(8,-90,0.35,-90);
                    currentStage = stage._290_Arm_Pickup_Tank3;
                }
                    break;
            case _290_Arm_Pickup_Tank3:
                if(robot.driveTrain.getCmdComplete()) {
                    //robot.driveTrain.CmdDrive(0, 0, 0.35, -90);
                   // robot.arm.setCurrentMode(Arm.Mode.PICKUP_TANK);
                    currentStage = stage._300_Snuffle5;
                }
                break;

            case _300_Snuffle5:
                if(robot.driveTrain.getCmdComplete()) {
                    //robot.driveTrain.CmdDrive(0, 0, 0.35, -90);
                    runtime.reset();
                    currentStage = stage._310_Snuffle6;
                }
                break;

            case _310_Snuffle6:
                if(robot.intake.BTargetFound || (runtime.milliseconds() >= 900)){
                    //robot.driveTrain.CmdDrive(24,-180,0.35,-90);
                    robot.arm.setCurrentMode(Arm.Mode.INTERMEDIATE);
                    currentStage = stage._313_BackWard;

                }

                else{
                    robot.arm.updateExtension(1.00);
                }

                break;
            case _313_BackWard:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(8,90,0.35,-90);
                    currentStage = stage._315_Turn3;
                }


                break;
            case _315_Turn3:
                if(robot.arm.getCmdComplete()){
                    robot.driveTrain.cmdTurn(-142,0.35);
                    currentStage = stage._320_Delivery3;
                }

                break;
            case _320_Delivery3:
                if (robot.driveTrain.getCmdComplete())     {
                    //robot.driveTrain.CmdDrive(0,-180,0.35,-90);
                    robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_BASKET);
                    currentStage = stage._330_Scoring_Drive3;
                }

                break;
            case _330_Scoring_Drive3:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(4,-150,0.35,-142);
                    currentStage = stage._340_Scoring_Drive3_2;
                }

                break;
            case _340_Scoring_Drive3_2:
                if (robot.driveTrain.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(16,-150,0.60,-142);
                    currentStage = stage._345_Scoring_Drive3_3;
                }

                break;
            case _345_Scoring_Drive3_3:
                if (robot.driveTrain.getCmdComplete() && robot.arm.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(4,-150,0.35,-142);
                    runtime.reset();
                    currentStage = stage._347_Pause4;
                }

                break;

            case _347_Pause4:
                if (robot.driveTrain.getCmdComplete()){
                    runtime.reset();
                    currentStage = stage._350_Output4;
                }
                break;

            case _350_Output4:
                if (runtime.milliseconds() >= 400)  {
                    //500 was 1000;
                    //robot.driveTrain.CmdDrive(8,90,0.35,-90);
                    robot.intake.doOut();
                    runtime.reset();
                    currentStage = stage._360_Drive_Right3;
                }

                break;
            case _360_Drive_Right3:
                if (runtime.milliseconds() >= 500)  {
                    robot.driveTrain.CmdDrive(6,15,0.35,-142);
                    currentStage = stage._370_Arm_Inter3;
                }

                break;
            case _370_Arm_Inter3:
                if (robot.driveTrain.getCmdComplete()){
                    // robot.driveTrain.CmdDrive(47,-180,0.35,-90);
                    robot.arm.setCurrentMode(Arm.Mode.INTERMEDIATE);
                    currentStage = stage._375_Turn4;
                }


                break;
            case _375_Turn4:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdTurn(0,0.35);
                    currentStage = stage._380_Forward4;
                }

                break;
            case _380_Forward4:
                if (robot.arm.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(44,25,0.65,0);
                    robot.arm.setCurrentMode(Arm.Mode.PICKUP_GROUND);
                    currentStage = stage._390_Drive_Right3;
                }



                break;
            case _390_Drive_Right3:
                if(robot.arm.getCmdComplete()){
                    robot.driveTrain.CmdDrive(22,90,0.45,0);
                    robot.arm.setCurrentMode(Arm.Mode.START);
                    currentStage = stage._400_TOUCHY;


                }

                break;

            case _400_TOUCHY:
                if(robot.driveTrain.getCmdComplete()){
                    robot.thePointyStick.setTouchy();
                    runtime.reset();
                    currentStage = stage._410_End;


                }

                break;
            case _410_End:
                if(runtime.milliseconds() >= 500){
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
        _10_Forward0,
        _20_TurnL,
        _25_Forward0_5,
        _30_Deliver_To_High_Basket,
        _35_Reverse0,
        _40_BearL1,
        _48_Pause,
        _50_OutPut,
        _60_BearR1,
        _70_Arm_Retract,
        _80_Forward1,
        _82_Forward1_2,
        _84_Forward1_3,
        _85_Arm_Pickup_Tank1,
        _87_Snuffle,
        _90_Snuffle2,
        _95_Turn1,
        _100_Delivery1,
        _110_Scoring_Drive1,
        _112_Scoring_Drive1_2,
        _114_Scoring_Drive1_3,
        _117_Pause2,
        _120_Output2,
        _123_Turn502,
        _125_Drive_Out1,
        _130_Arm_Inter,
        _140_Drive_Out1_2,
        _145_Turn1_1,
        _150_Drive_Out1_3,
        _160_Forward2_3,
        _170_Arm_Pickup_Tank2,
        _180_Snuffle3,
        _190_Snuffle4,
        _195_Turn2,
        _200_Delivery2,
        _210_Scoring_Drive2,
        _220_Scoring_Drive2_2,
        _225_Scoring_Drive1_3,
        _227_Pause3,
        _230_Output3,
        _240_Drive_Out3,
        _250_Arm_Inter2,
        _260_Drive_Out3_2,
        _265_Turn2_2,
        _270_Drive_Out3_3,
        _280_Drive_left8,
        _290_Arm_Pickup_Tank3,
        _300_Snuffle5,
        _310_Snuffle6,
        _313_BackWard,
        _315_Turn3,
        _320_Delivery3,
        _330_Scoring_Drive3,
        _340_Scoring_Drive3_2,
        _345_Scoring_Drive3_3,
        _347_Pause4,
        _350_Output4,
        _360_Drive_Right3,
        _370_Arm_Inter3,
        _375_Turn4,
        _380_Forward4,
        _390_Drive_Right3,
        _400_TOUCHY,
        _410_End


    }
}