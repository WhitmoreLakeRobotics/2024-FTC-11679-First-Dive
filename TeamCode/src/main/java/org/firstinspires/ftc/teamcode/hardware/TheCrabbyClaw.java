package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class TheCrabbyClaw extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    //private ColorRangeSensor IntakeSensor;
    //private DistanceSensor RearLeftSensor



    private boolean cmdComplete = true;
    private Mode CurrentMode = Mode.NOTGO;

    public Servo Tamatoa;

    public final double NotGoPos = 0.5;
    public final double ShinyPos = 0;
    public final double DropPos = 1;

    // now that i think about it do i need a stop position for this;
    //positions for this not known yet;



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
        Tamatoa = hardwareMap.get(Servo.class,"Tamatoa");
    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {
    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){

        switch(CurrentMode){
            case NOTGO:
                doNotGo();

                break;
            case DROP:

                break;
            case SHINY:

            //make case for each option ( copied from "Intake");
        }





    }

     public void doShiny(){
        CurrentMode = Mode.SHINY;
        Tamatoa.setPosition(ShinyPos);
        cmdComplete = true;
    }
    public void doDrop(){
        CurrentMode = Mode.DROP;
        Tamatoa.setPosition(DropPos);
        cmdComplete = true;
    }





    public void doNotGo(){
        CurrentMode = Mode.NOTGO;
        Tamatoa.setPosition(NotGoPos);
        cmdComplete = true;
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

    private enum Mode{
        SHINY,
        DROP,
        NOTGO;
    }





}

