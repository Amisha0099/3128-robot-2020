package org.team3128.compbot.subsystems;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.LinkedList;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.game_elements.Ball;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hopper extends Threaded {

    public enum HopperState {
        POS_0(new boolean[] { false, false, false, false }),
        POS_1(new boolean[] { false, false, false, true }),
        POS_2(new boolean[] { false, false, true, true }),
        POS_3(new boolean[] { false, true, true, true }),
        POS_4(new boolean[] { true, true, true, true });
        // POS_5(new boolean[] { true, true, true, true }),
        // POS_6(new boolean[] { false, true, true, true });

        public boolean[] hopperState;

        private HopperState(boolean[] hopperState) {
            this.hopperState = hopperState;
        }
    }

    public enum ActionState {
        STANDBY, INTAKING, SHOOTING, ORGANIZING;

        private ActionState() {

        }
    }

    public LazyCANSparkMax INTAKE_MOTOR, HOPPER_FEEDER_MOTOR, CORNER_MOTOR, GATEKEEPER_MOTOR;
    public CANEncoder HOPPER_FEEDER_ENCODER, CORNER_ENCODER;
    public DigitalInput SENSOR_0, SENSOR_1;

    boolean[] ballArray = { false, false, false, false };
    private static final Hopper instance = new Hopper();

    // private boolean isMoving;
    private boolean isShooting, isIntaking;
    private boolean empty0 = true;
    private boolean empty1 = true;
    private boolean isFeeding = false;
    private boolean isLoading = false;
    // private boolean isOrganizing = false;
    private boolean openTheGates = false;
    private double startPos = 0;
    private int ballCount;
    private String debugString = "lalala";
    private String arrayString = "";

    public ActionState actionState;

    private DigitalInput[] sensorPositions = { SENSOR_0, SENSOR_1 }; // top to bottom //0 = 1.5, 1 = 3, 2 = 4
                                                                     // mathhh

    public static Hopper getInstance() {
        return instance;
    }

    private Hopper() {
        configMotors();
        configEncoders();
        configSensors();
        ballCount = 0; // TODO: initial ball count
    }

    @Override
    public void update() {

        switch (actionState) {
            case STANDBY:
                if (!isFull() && !(detectsBall(SENSOR_0) && (ballCount >= 3))) {
                    standbyIntake();
                } else {
                    setMotorPowers(0, 0, 0);
                }
                break;

            case INTAKING:
                if (!isFull() && !(detectsBall(SENSOR_0) && (ballCount >= 3))) {
                    standbyIntake();
                } else {
                    setMotorPowers(0, 0, 0);
                }
                break;

            case SHOOTING:
                loadShoot();
                break;

            case ORGANIZING:
                organize();
                break;
        }

        SmartDashboard.putString("Hopper isFeeding", "" + isFeeding);
        //Log.info("Hopper", "actionState is " + actionState);
        //Log.info("Hopper", "fullness is " + isFull()); 
        arrayString = "";
        for (boolean ele : getBallArray()) {
            arrayString += String.valueOf(ele);
            arrayString += ", ";
        }
        SmartDashboard.putString("Hopper array", arrayString);
        SmartDashboard.putNumber("Hopper callBount", ballCount);
        //SmartDashboard.putString("Hopper feeder sensor function", "" + detectsBall(SENSOR_1));
        if (detectsBall(SENSOR_1)) {
            empty1 = false;
        } else {
            empty1 = true;
        }
        if (detectsBall(SENSOR_0)) {
            empty0 = false;
        } else {
            empty0 = true;
        }
        //Log.info("Hopper", String.valueOf(actionState));
        //Log.info("Hopper", String.valueOf(detectsBall(SENSOR_1)));
    }

    private void configMotors() {
        INTAKE_MOTOR = new LazyCANSparkMax(Constants.IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        HOPPER_FEEDER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.HOPPER_FEEDER_MOTOR_ID,
                MotorType.kBrushless);
        CORNER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.CORNER_MOTOR_ID, MotorType.kBrushless);
        GATEKEEPER_MOTOR = new LazyCANSparkMax(Constants.HopperConstants.GATEKEEPER_MOTOR_ID, MotorType.kBrushless);
    }

    private void configEncoders() {
        HOPPER_FEEDER_ENCODER = HOPPER_FEEDER_MOTOR.getEncoder();
        CORNER_ENCODER = CORNER_MOTOR.getEncoder();
    }

    private void configSensors() {
        SENSOR_0 = new DigitalInput(Constants.HopperConstants.SENSOR_0_ID);
        SENSOR_1 = new DigitalInput(Constants.HopperConstants.SENSOR_1_ID);
    }

    public void setBallOrder(boolean[] CurrBallArray) {
        if (CurrBallArray.length == ballArray.length) {
            for (int i = 0; i < ballArray.length; i++) {
                ballArray[i] = CurrBallArray[i];
            }
        }
    }

    public boolean isReady() {
        return true; // TODO: only return true if there is a ball ready to be shot AND the shoot
                      // method isn't currently in use.
    }

    public int getNumBalls() {
        int ball_count = 0;
        for (boolean ball : ballArray) {
            if (ball) {
                ball_count++;
            }
        }
        return ball_count;
    }

    public boolean isEmpty() {
        return (getNumBalls() == 0);
        //return false;
    }

    public boolean isFull() {
        return (getNumBalls() == Constants.HopperConstants.CAPACITY - 1);
    }

    public boolean[] shift(boolean[] in_array) {
        boolean[] out_array = new boolean[in_array.length];
        for (int i = 0; i < in_array.length - 1; i++) {
            out_array[i] = in_array[i + 1];
        }
        out_array[in_array.length - 1] = false;
        return out_array;
    }

    private boolean[] addBall(boolean[] in_array) {
        boolean[] out_array = new boolean[in_array.length];
        // for (int i = 0; i < in_array.length; i++) {
        boolean added = false;
        for (int i = in_array.length - 1; i >= 0; i--) {
            if (in_array[i]) {
                out_array[i] = in_array[i];
            } else if (!added) {
                out_array[i] = true;
                added = true;
            }
        }
        return out_array;
    }

    public boolean[] getBallArray() {
        return ballArray;
    }

    public void updateBallArray(boolean[] in_array) {
        this.ballArray = in_array;
    }

    // public void setIsMoving(boolean isMoving) {
    // this.isMoving = isMoving;
    // }

    public int getFirstAvailablePos() {
        int tempCount = 0;
        int record_holder_index = 0;
        for (boolean ball : ballArray) {
            if (ball) {
                record_holder_index = tempCount;
            }
            tempCount++;
        }
        return record_holder_index + 1;
    }

    public void setMotorPowers(double p_Gatekeeer, double p_Corner, double p_HopperFeeder) {
        GATEKEEPER_MOTOR.set(p_Gatekeeer);
        CORNER_MOTOR.set(p_Corner);
        HOPPER_FEEDER_MOTOR.set(p_HopperFeeder);
    }

    public void gateKeep(boolean value) {
        openTheGates = value;
    }

    // public void setIsOrganizing(boolean value) {
    // isOrganizing = value;
    // }

    public void shoot() {
        gateKeep(true);
    }

    public boolean detectsBall(DigitalInput sensor) {
        return !sensor.get();
    }

    public void setAction(ActionState state) {
        this.actionState = state;
        if (state == ActionState.INTAKING) {
            INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_ON_VALUE);
        } else {
            INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_OFF_VALUE);
        }
    }

    public void standbyIntake() { //run if we aren't saturated with balls and we are in INTAKING or STANDBY states

        if (detectsBall(SENSOR_1)) { //if there is a ball in the first sensor position
            empty1 = false; //tell code the position isn't empty
            if (!isFeeding) { //if we aren't trying to move the intaken ball into the lowest position
                setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER); //just move all the motors
            } else {
                setMotorPowers(0, Constants.HopperConstants.BASE_POWER, 0); //otherwise, only move the corner motor move the intaken ball into the lowest position
            }
            Log.info("Hopper", "detects ball");
        } else if (!empty1) { //if there isn't a ball in the first position, but there was one in the last iteration
            empty1 = true; //tell the code the position is empty
            ballCount++; //iterate ballCount once because a ball has passed through our sensors

            updateBallArray(addBall(getBallArray())); //also fix the array

            startPos = CORNER_ENCODER.getPosition(); //record the current corner motor encoder position
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, 0); //only move the corner motor to move the ball into the lowest position
            isFeeding = true; //tell the code we are trying move this ball into the lowest position
            Log.info("Hopper", "empty is true");
        } else if (!isFeeding) {
            setMotorPowers(0, 0, 0);
            //Log.info("Hopper", "setting motor powers to 0");
        } else { //if we are trying to move the ball into the lowest position
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, 0); //only move the corner motor
            Log.info("Hopper", "" + CORNER_ENCODER.getPosition());
            if (Math.abs(CORNER_ENCODER.getPosition() - Constants.HopperConstants.BALL_SPACING[ballCount - 1]) >= Math.abs(startPos)) { //if the ball gets to the right position
                isFeeding = false; //we're done
                Log.info("Hopper", "reached end of offset");
            }
            Log.info("Hopper", "is feeding");
        }
    }

    public void loadShoot() { //loading the balls to be shot when in SHOOTING state
        if (!detectsBall(SENSOR_0)) { //if there is no ball in Sensor 0 location
            if (!empty0 && openTheGates) { //if there was a ball in Sensor 0 in the prior iteration and we have opened the gates
                ballCount--; //decrease ballCount by 1
                updateBallArray(shift(getBallArray())); //fix array
                openTheGates = false; //gatekeeper off
            }
            empty0 = true; //set to empty
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER); //shift all of the balls forward until there is a ball in Sensor position 0
        } else if (empty0) { //if there is a ball in Sensor position 0 and there wasn't in the prior iteration
            empty0 = false; //it's no longer empty

            startPos = CORNER_ENCODER.getPosition(); //record current motor encoder position
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER); //continue moving all the balls forward for the set offset distance
            isLoading = true; //tell the code we are moving the balls forward for the set offset distance
        } else if (isLoading) { //if we are trying to move the ball forward an offset
            setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER); //shift everything the offset
            if (Math.abs(CORNER_ENCODER.getPosition() - Constants.HopperConstants.SHOOTER_SPACING) >= Math.abs(startPos)) { //if the offset has been reached
                isLoading = false; //stop
            }
        } else if (openTheGates) { //if there is a ball in sensor position 0 and we aren't trying to move that ball forward for the offset distance and we are shooting the ball
            setMotorPowers(Constants.HopperConstants.GATEKEEPER_POWER, Constants.HopperConstants.BASE_POWER,
                    Constants.HopperConstants.BASE_POWER); //move everything
            Log.info("Hopper", "opening the gates");
        }
        if (detectsBall(SENSOR_1)) { //if there is a ball in sensor 1
            empty1 = false; //tell the code
        } else if (!empty1) { //if there is no ball in sensor 1 but there was in the prior iteration
            empty1 = true; //tell the code
            ballCount++; //iterate ball count (a ball has moved into the hopper)
            updateBallArray(addBall(getBallArray())); //fix array
        }
    }

    public void organize() {
        if (isEmpty()) {
            setAction(ActionState.STANDBY);
        }
        if (!detectsBall(SENSOR_1)) {
            empty1 = true;
            setMotorPowers(0, -Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
        } else if (empty1) {
            empty1 = false;
            setAction(ActionState.STANDBY);
        }
    }
}
