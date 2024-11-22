package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.CommonLogic;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Arm extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    //private ColorRangeSensor IntakeSensor;
    //private DistanceSensor RearLeftSensor

    private int EMIntol = 50;
    private int AMIntol = 50;
    private boolean EMINPos = true;
    private boolean AMINPos = true;
    private boolean cmdComplete = true;
    private Mode CurrentMode = Mode.STOP;
    private DcMotor AM1;
    private DcMotor EM1;
    private DcMotor AM2;
    private DcMotor EM2;
    private Servo NTKA1;

    private double sOUT = 0.4;
    private double sUP = 0.0;
    private double sDOWN = 1.0;

    private static final double ARMSPEED = 0.75;
    private double ARMHOLDPOWER =0.00;
    private static final int minArmPos = 0;
    private static final int maxArmPos = 1690;
    private double armPValue = 50;
    private int armTargetPos = 0;

    private static final double EXTSPEED = 1.00;
    private double EXTHOLDPOWER =0.00;
    private static final int minExtPos = 0;
    private static final int maxExtPos = 3090;
    private double extPValue = 50;
    private int extTargetPos = 0;
    private int extMaxPos;

 private Servo NTKS02;


    private final double StartPos = 0;
    private final double CarryPos = 0.3;
    private final double NTKPos = 0.7;
    private final double RvsSlam = 1;

    private static double extStepSize = 50.0;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    /*public Swing_Arm_And_Lift() {

    }*/

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init(){
        AM1 = hardwareMap.dcMotor.get("AM1");
        AM1.setDirection(DcMotor.Direction.REVERSE);
       // AM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // AM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        EM1 = hardwareMap.dcMotor.get("EM1");
        EM1.setDirection(DcMotor.Direction.REVERSE);
       // EM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // EM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        EM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        AM2 = hardwareMap.dcMotor.get("AM2");
        AM2.setDirection(DcMotor.Direction.FORWARD);
       // AM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // AM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        EM2 = hardwareMap.dcMotor.get("EM2");
        EM2.setDirection(DcMotor.Direction.FORWARD);
       // EM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // EM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        EM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        NTKA1 = hardwareMap.get(Servo.class,"NTKA1");


    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {

       telemetry.addData("AM1 ",AM1.getCurrentPosition());
       telemetry.addData("EM1 ",EM1.getCurrentPosition());
        telemetry.addData("AM2 ",AM2.getCurrentPosition());
        telemetry.addData("EM2 ",EM2.getCurrentPosition());
       telemetry.addData( " Arm Mode ", CurrentMode.toString());
    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){
       // setWristOut();
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){
        telemetry.addData("AM1 ",AM1.getCurrentPosition());
        telemetry.addData("EM1 ",EM1.getCurrentPosition());
        telemetry.addData("AM2 ",AM2.getCurrentPosition());
        telemetry.addData("EM2 ",EM2.getCurrentPosition());
        telemetry.addData( " Arm Mode ", CurrentMode.toString());

        if((CommonLogic.inRange(AM1.getCurrentPosition(),armTargetPos,AMIntol) ) &&
                ((CommonLogic.inRange(EM1.getCurrentPosition(),extTargetPos,EMIntol)) ))
        {
            cmdComplete = true;
        }
        else{
            cmdComplete = false;
        }

        AM1.setPower(CommonLogic.CapValue(
                CommonLogic.PIDcalc(armPValue,ARMHOLDPOWER,AM1.getCurrentPosition(),armTargetPos)
                ,-ARMSPEED,ARMSPEED));

        AM2.setPower(CommonLogic.CapValue(
                CommonLogic.PIDcalc(armPValue,ARMHOLDPOWER,AM2.getCurrentPosition(),armTargetPos)
                ,-ARMSPEED,ARMSPEED));

        EM1.setPower(CommonLogic.CapValue(
                CommonLogic.PIDcalc(extPValue,EXTHOLDPOWER,EM1.getCurrentPosition(),extTargetPos)
                ,-EXTSPEED,EXTSPEED));

        EM2.setPower(CommonLogic.CapValue(
                CommonLogic.PIDcalc(extPValue,EXTHOLDPOWER,EM2.getCurrentPosition(),extTargetPos)
                ,-EXTSPEED,EXTSPEED));

        switch(CurrentMode){
            case IDLE:

                break;
            case START:
                armTargetPos = CommonLogic.CapValueint(Mode.START.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.START.ArmP;
                ARMHOLDPOWER = Mode.START.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.START.ExtPos, Mode.START.ExtPos,Mode.START.ExtMax);
                extPValue = Mode.START.ExtP;
                EXTHOLDPOWER = Mode.START.ExtF;
                extMaxPos = Mode.START.ExtMax;
                CurrentMode = Mode.IDLE;

                break;
            case CLIMB:
                armTargetPos = CommonLogic.CapValueint(Mode.CLIMB.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.CLIMB.ArmP;
                ARMHOLDPOWER = Mode.CLIMB.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.CLIMB.ExtPos, Mode.CLIMB.ExtPos,Mode.CLIMB.ExtMax);
                extPValue = Mode.CLIMB.ExtP;
                EXTHOLDPOWER = Mode.CLIMB.ExtF;
                extMaxPos = Mode.CLIMB.ExtMax;
                CurrentMode = Mode.IDLE;


                break;
            case PICKUP_TANK:
                armTargetPos = CommonLogic.CapValueint(Mode.PICKUP_TANK.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.PICKUP_TANK.ArmP;
                ARMHOLDPOWER = Mode.PICKUP_TANK.ArmF;

                //extTargetPos = CommonLogic.CapValueint(Mode.PICKUP_TANK.ExtPos, Mode.PICKUP_TANK.ExtPos,Mode.PICKUP_TANK.ExtMax);
                extPValue = Mode.PICKUP_TANK.ExtP;
                EXTHOLDPOWER = Mode.PICKUP_TANK.ExtF;
                extMaxPos = Mode.PICKUP_TANK.ExtMax;
                CurrentMode = Mode.IDLE;



                break;
            case PICKUP_WALL:
                armTargetPos = CommonLogic.CapValueint(Mode.PICKUP_WALL.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.PICKUP_WALL.ArmP;
                ARMHOLDPOWER = Mode.PICKUP_WALL.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.PICKUP_WALL.ExtPos, Mode.PICKUP_WALL.ExtPos,Mode.PICKUP_WALL.ExtMax);
                extPValue = Mode.PICKUP_WALL.ExtP;
                EXTHOLDPOWER = Mode.PICKUP_WALL.ExtF;
                extMaxPos = Mode.PICKUP_WALL.ExtMax;
                CurrentMode = Mode.IDLE;


                break;
            case PICKUP_GROUND:
                armTargetPos = CommonLogic.CapValueint(Mode.PICKUP_GROUND.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.PICKUP_GROUND.ArmP;
                ARMHOLDPOWER = Mode.PICKUP_GROUND.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.PICKUP_GROUND.ExtPos, Mode.PICKUP_GROUND.ExtPos,Mode.PICKUP_GROUND.ExtMax);
                extPValue = Mode.PICKUP_GROUND.ExtP;
                EXTHOLDPOWER = Mode.PICKUP_GROUND.ExtF;
                extMaxPos = Mode.PICKUP_GROUND.ExtMax;
                CurrentMode = Mode.IDLE;


                break;
            case DELIVER_TO_LOW_BASKET:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_LOW_BASKET.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.DELIVER_TO_LOW_BASKET.ArmP;
                ARMHOLDPOWER = Mode.DELIVER_TO_LOW_BASKET.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_LOW_BASKET.ExtPos, Mode.DELIVER_TO_LOW_BASKET.ExtPos,Mode.DELIVER_TO_LOW_BASKET.ExtMax);
                extPValue = Mode.DELIVER_TO_LOW_BASKET.ExtP;
                EXTHOLDPOWER = Mode.DELIVER_TO_LOW_BASKET.ExtF;
                extMaxPos = Mode.DELIVER_TO_LOW_BASKET.ExtMax;
                CurrentMode = Mode.IDLE;


                break;
            case DELIVER_TO_HIGH_BASKET:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_HIGH_BASKET.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.DELIVER_TO_HIGH_BASKET.ArmP;
                ARMHOLDPOWER = Mode.DELIVER_TO_HIGH_BASKET.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_HIGH_BASKET.ExtPos, Mode.DELIVER_TO_HIGH_BASKET.ExtPos,Mode.DELIVER_TO_HIGH_BASKET.ExtMax);
                extPValue = Mode.DELIVER_TO_HIGH_BASKET.ExtP;
                EXTHOLDPOWER = Mode.DELIVER_TO_HIGH_BASKET.ExtF;
                extMaxPos = Mode.DELIVER_TO_HIGH_BASKET.ExtMax;
                CurrentMode = Mode.IDLE;


                break;
            case CLIMB2:
                armTargetPos = CommonLogic.CapValueint(Mode.CLIMB2.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.CLIMB2.ArmP;
                ARMHOLDPOWER = Mode.CLIMB2.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.CLIMB2.ExtPos, Mode.CLIMB2.ExtPos,Mode.CLIMB2.ExtMax);
                extPValue = Mode.CLIMB2.ExtP;
                EXTHOLDPOWER = Mode.CLIMB2.ExtF;
                extMaxPos = Mode.CLIMB2.ExtMax;
                CurrentMode = Mode.IDLE;


                break;
            case INTERMEDIATE:
                armTargetPos = CommonLogic.CapValueint(Mode.INTERMEDIATE.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.INTERMEDIATE.ArmP;
                ARMHOLDPOWER = Mode.INTERMEDIATE.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.INTERMEDIATE.ExtPos, Mode.INTERMEDIATE.ExtPos,Mode.INTERMEDIATE.ExtMax);
                extPValue = Mode.INTERMEDIATE.ExtP;
                EXTHOLDPOWER = Mode.INTERMEDIATE.ExtF;
                extMaxPos = Mode.INTERMEDIATE.ExtMax;
                CurrentMode = Mode.IDLE;


                break;
            case DELIVER_TO_OBSERVATION:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_OBSERVATION.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.DELIVER_TO_OBSERVATION.ArmP;
                ARMHOLDPOWER = Mode.DELIVER_TO_OBSERVATION.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_OBSERVATION.ExtPos, Mode.DELIVER_TO_OBSERVATION.ExtPos,Mode.DELIVER_TO_OBSERVATION.ExtMax);
                extPValue = Mode.DELIVER_TO_OBSERVATION.ExtP;
                EXTHOLDPOWER = Mode.DELIVER_TO_OBSERVATION.ExtF;
                extMaxPos = Mode.DELIVER_TO_OBSERVATION.ExtMax;
                CurrentMode = Mode.IDLE;


                break;
            case DELIVER_TO_HIGH_CHAMBER:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_HIGH_CHAMBER.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.DELIVER_TO_HIGH_CHAMBER.ArmP;
                ARMHOLDPOWER = Mode.DELIVER_TO_HIGH_CHAMBER.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_HIGH_CHAMBER.ExtPos, Mode.DELIVER_TO_HIGH_CHAMBER.ExtPos,Mode.DELIVER_TO_HIGH_CHAMBER.ExtMax);
                extPValue = Mode.DELIVER_TO_HIGH_CHAMBER.ExtP;
                EXTHOLDPOWER = Mode.DELIVER_TO_HIGH_CHAMBER.ExtF;
                extMaxPos = Mode.DELIVER_TO_HIGH_CHAMBER.ExtMax;
                CurrentMode = Mode.IDLE;


                break;
            case TANK_ENTRY:
                armTargetPos = CommonLogic.CapValueint(Mode.TANK_ENTRY.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.TANK_ENTRY.ArmP;
                ARMHOLDPOWER = Mode.TANK_ENTRY.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.TANK_ENTRY.ExtPos, Mode.TANK_ENTRY.ExtPos,Mode.TANK_ENTRY.ExtMax);
                extPValue = Mode.TANK_ENTRY.ExtP;
                EXTHOLDPOWER = Mode.TANK_ENTRY.ExtF;
                extMaxPos = Mode.TANK_ENTRY.ExtMax;
                CurrentMode = Mode.IDLE;


                break;
            case STOP:
                armTargetPos = AM1.getCurrentPosition();
                armPValue = Mode.STOP.ArmP;
                ARMHOLDPOWER = Mode.STOP.ArmF;

                extTargetPos = EM1.getCurrentPosition();
                extPValue = Mode.STOP.ExtP;
                EXTHOLDPOWER = Mode.STOP.ExtF;
                extMaxPos = Mode.STOP.ExtMax;
                CurrentMode = Mode.IDLE;


                break;
            default:
        }


    }

    public void doStop(){
        CurrentMode = Mode.STOP;

        cmdComplete = true;
    }



    public boolean getCmdComplete(){
        return cmdComplete;
    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    void stop(){

    }

    public void resetEncoders(){

        AM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        EM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        EM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void updateExtension(double updateTarget){
        //multiply update target by some amount then add to target pos.
        //wrap set value in cap
        int nTarget = (int) (extTargetPos + (updateTarget * extStepSize));
        extTargetPos = CommonLogic.CapValueint(nTarget,0,extMaxPos);
    }

    public void setCurrentMode(Mode Nmode){
        //update to recieve and set mode
    CurrentMode = Nmode;
    }

    public Mode getCurrentMode(){
        return  CurrentMode;
    }

    public void setWristUp(){NTKA1.setPosition(sUP);}
    public void setWristOut(){NTKA1.setPosition(sOUT);}
    public void setWristDown(){NTKA1.setPosition(sDOWN);}


    public enum Mode{
        START(0,100,0,0,100,0,5),
        PICKUP_TANK(0,120,0,560,100,0,1716),
        PICKUP_GROUND(0,100,0,138,100,0,1716),
        PICKUP_WALL(15,100,0,0,100,0,5),
        DELIVER_TO_OBSERVATION(20,100,0,0,100,0,5),
        CLIMB2(1000,100,0,1457,100,0,1460),
        DELIVER_TO_HIGH_CHAMBER(30,100,0,0,100,0,5),
        DELIVER_TO_LOW_BASKET(1200,100,0,3000,100,0,3020),
        //DELIVER_TO_LOW_BASKET(1144,100,0,3000,100,0,3020),
        DELIVER_TO_HIGH_BASKET(1450,100,0.15,2500,100,0.05,2700),
        CLIMB(1300,150,0,2500,100,0,3000),
       // CLIMB(1000,100,0,1457,100,0,1460),
        STOP(0,1000000000,0,0,10000000,0,5),
        INTERMEDIATE(1500, 120, 0, 500, 120, 0, 1600),
        TANK_ENTRY(400,150,0,560,100,0,1716),
        IDLE(0,0,0,0,0,0, 0);


        private int ArmPos;
        private double ArmP;
        private double ArmF;
        private int ExtPos;
        private double ExtP;
        private double ExtF;
        private int ExtMax;

        private Mode(int ArmPos,double ArmP, double ArmF,int ExtPos, double ExtP,double ExtF, int ExtMax){
            this.ArmPos = ArmPos;
            this.ArmP = ArmP;
            this.ArmF = ArmF;
            this.ExtPos = ExtPos;
            this.ExtP = ExtP;
            this.ExtF = ExtF;
            this.ExtMax = ExtMax;
        }



    }





}

