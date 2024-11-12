package org.firstinspires.ftc.teamcode.hardware;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.Callable;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class GrapplingHook extends BaseHardware {

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
    private Mode CurrentMode = Mode.STOP;
    public Servo GHS01;
    public Servo GHS02;
    public ColorRangeSensor BCS01;

    private static double TargetDistance = 1.15; //in inches;

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

        GHS01 = hardwareMap.get(Servo.class,"GHS01");
        GHS02 = hardwareMap.get(Servo.class,"GHS02");

        GHS01.setPosition( 0.5 );
        GHS02.setPosition( 0.5 );

        BCS01 = hardwareMap.get(ColorRangeSensor.class, "BCS01");
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

    }

    public void doStop(){
        CurrentMode = Mode.STOP;

        cmdComplete = true;
    }

    public void updateServo( double y ){
        double x = y/2 + 0.5;     // converting joystick position to servo position;
        double x2 = -y/2 + 0.5;
        GHS01.setPosition( x2 );
        GHS02.setPosition( x );

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

    public boolean TheFloorIsLava(){
        if (BCS01.getDistance(DistanceUnit.INCH) >= TargetDistance){
            return true;
        }
        else{
            return false;
        }

    }


    private enum Mode{
        STOP;
    }









}

