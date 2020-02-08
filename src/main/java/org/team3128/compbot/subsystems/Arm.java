package org.team3128.compbot.subsystems;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;

public class Arm extends Threaded {
    public enum ArmState {
        STOWED(0), // arm is all the way down
        INTAKE(0), // intaking balls
        STARTING(45), // within frame perimeter
        FAR_RANGE(60), // far range shooting
        SHORT_RANGE(20); // short range shooting

        public double armAngle;

        private ArmState(double angle) {
            this.armAngle = angle;
        }
    }

    public static final Arm instance = new Arm();
    LazyCANSparkMax ARM_MOTOR_LEADER, ARM_MOTOR_FOLLOWER;
    CANEncoder ARM_ENCODER;
    DigitalInput LIMIT_SWITCH;
    double setpoint;
    double current = 0;
    double error = 0;
    double output = 0;
    double accumulator = 0;
    double prevError = 0;
    ArmState ARM_STATE;
    boolean isZeroing = false;

    public Arm getInstance() {
        return instance;
    }

    private Arm() {
        configMotors();
        configEncoders();
        configSensors();
        setState(ArmState.STARTING);
    }

    private void configSensors() {
        LIMIT_SWITCH = new DigitalInput(Constants.ARM_LIMIT_SWITCH_ID)
    }

    private void configEncoders() {
        ARM_ENCODER = ARM_MOTOR_LEADER.getEncoder();
    }

    private void configMotors() {
        ARM_MOTOR_LEADER = new LazyCANSparkMax(Constants.ARM_MOTOR_LEADER_ID, MotorType.kBrushless);
        ARM_MOTOR_FOLLOWER = new LazyCANSparkMax(Constants.ARM_MOTOR_FOLLOWER_ID, MotorType.kBrushless);

        ARM_MOTOR_FOLLOWER.follow(ARM_MOTOR_LEADER);

        ARM_MOTOR_LEADER.setNeutralMode(Constants.ARM_NEUTRAL_MODE);
        ARM_MOTOR_FOLLOWER.setNeutralMode(Constants.ARM_NEUTRAL_MODE);
    }

    private void setSetpoint(double desiredPos) {
        setpoint = desiredPos;
    }

    public void setState(ArmState armState) {
        ARM_STATE = armState;
        setSetpoint(armState.armAngle);
    }

    private double armFeedForward(double desired) {
        return 0; // TODO: add feedforward implementation for arm control
    }

    public void zero() {
        isZeroing = true;
    }

    @Override
    public void update() {
        if (isZeroing) {
            if (!LIMIT_SWITCH.get()) {
                ARM_MOTOR_LEADER.set(ControlMode.Percent, Constants.ZEROING_POWER);
            }
        } else {

            if (ARM_STATE.armAngle != setpoint) {
                Log.info("ARM", "Setpoint override (setpoint has been set without using ArmState)");
            }
            current = ((ARM_ENCODER.getPosition() / Constants.ARM_GEARING) * 360) % 360; // TODO: account for possible
                                                                                        // negative values
            error = setpoint - current;
            accumulator += error * Constants.DT;
            if (accumulator > Constants.ARM_SATURATION_LIMIT) {
                accumulator = Constants.ARM_SATURATION_LIMIT;
            } else if (accumulator < -Constants.ARM_SATURATION_LIMIT) {
                accumulator = -Constants.ARM_SATURATION_LIMIT;
            }
            double kP_term = Constants.kP_ARM * error;
            double kI_term = Constants.kI_ARM * accumulator;
            double kD_term = Constants.kD_ARM * (error - prevError) / Constants.DT;

            double voltage_output = armFeedForward(setpoint) + kP_term + kI_term + kD_term;
            double voltage = RobotController.getBatteryVoltage();

            output = voltage_output / voltage;
            if (output > 1) {
                Log.info("ARM",
                        "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this");
                output = 1;
            } else if (output < -1) {
                Log.info("ARM",
                        "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this ");
                output = -1;
            }

            ARM_MOTOR_LEADER.set(ControlMode.Percent, output);

            prevError = error;
    }

}