package org.team3128.compbot.subsystems;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonFX;

public class Arm extends Threaded {
    public enum ArmState {
        STOWED(0), // arm is all the way down
        INTAKE(0), // intaking balls
        STARTING(15), // within frame perimeter
        STARTING_DOWN(30),
        FAR_RANGE(60), // far range shooting
        SHORT_RANGE(20); // short range shooting

        public double armAngle;

        private ArmState(double angle) {
            this.armAngle = angle;
        }
    }

    public static final Arm instance = new Arm();
    public LazyTalonFX ARM_MOTOR_LEADER, ARM_MOTOR_FOLLOWER;
    public DigitalInput LIMIT_SWITCH;
    double setpoint;
    double current = 0;
    double error = 0;
    public double output = 0;
    double accumulator = 0;
    double prevError = 0;
    public ArmState ARM_STATE;
    boolean isZeroing = false;
    public int plateauCount = 0;

    public static Arm getInstance() {
        return instance;
    }

    private Arm() {
        configMotors();
        configSensors();
        setState(ArmState.STARTING);
    }

    private void configSensors() {
        LIMIT_SWITCH = new DigitalInput(Constants.ArmConstants.ARM_LIMIT_SWITCH_ID);
    }

    private void configMotors() {
        ARM_MOTOR_LEADER = new LazyTalonFX(Constants.ArmConstants.ARM_MOTOR_LEADER_ID);
        ARM_MOTOR_FOLLOWER = new LazyTalonFX(Constants.ArmConstants.ARM_MOTOR_FOLLOWER_ID);

        ARM_MOTOR_FOLLOWER.follow(ARM_MOTOR_LEADER);

        ARM_MOTOR_LEADER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
        ARM_MOTOR_FOLLOWER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);

        ARM_MOTOR_LEADER.setSelectedSensorPosition(0);
        ARM_MOTOR_LEADER.setSensorPhase(true);
    }

    private void setSetpoint(double desiredPos) {
        setpoint = desiredPos;
    }

    public void setState(ArmState armState) {
        ARM_STATE = armState;
        setSetpoint(armState.armAngle);
    }

    public double armFeedForward(double desired) {
        return 0; // true value = -0.46
    }

    public double getAngle() {
        return (((getEncoderPos() / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                / Constants.ArmConstants.ARM_GEARING) * 360) % 360; // TODO: account for
        // possible negative
    }

    public void zero() {
        setState(ArmState.STOWED);
    }

    private double getEncoderPos() {
        return ARM_MOTOR_LEADER.getSelectedSensorPosition(0);
    }

    private double getEncoderVel() {
        return ARM_MOTOR_LEADER.getSelectedSensorVelocity(0);
    }

    public boolean getLimitStatus() {
        return LIMIT_SWITCH.get();
    }

    @Override
    public void update() {
        if (setpoint > Constants.ArmConstants.MAX_ARM_ANGLE) {
            setpoint = Constants.ArmConstants.MAX_ARM_ANGLE;
        }

        if (setpoint < 0) {
            setpoint = 0;
        }

        if (!getLimitStatus()) {
            ARM_MOTOR_LEADER.setSelectedSensorPosition(0);
            ARM_MOTOR_FOLLOWER.setSelectedSensorPosition(0);
        }

        if (ARM_STATE.armAngle != setpoint) {
            Log.info("ARM", "Setpoint override (setpoint has been set without using ArmState)");
        }
        current = getAngle();
        error = setpoint - current;
        accumulator += error * Constants.MechanismConstants.DT;
        if (accumulator > Constants.ArmConstants.ARM_SATURATION_LIMIT) {
            accumulator = Constants.ArmConstants.ARM_SATURATION_LIMIT;
        } else if (accumulator < -Constants.ArmConstants.ARM_SATURATION_LIMIT) {
            accumulator = -Constants.ArmConstants.ARM_SATURATION_LIMIT;
        }
        double kP_term = Constants.ArmConstants.ARM_PID.kP * error;
        double kI_term = Constants.ArmConstants.ARM_PID.kI * accumulator;
        double kD_term = Constants.ArmConstants.ARM_PID.kD * (error - prevError) / Constants.MechanismConstants.DT;

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

        if (Math.abs(error) < Constants.ArmConstants.ANGLE_THRESHOLD) {
            plateauCount++;
        } else {
            plateauCount = 0;
        }

        ARM_MOTOR_LEADER.set(ControlMode.PercentOutput, output);

        prevError = error;
    }

    public boolean isReady() {
        return (plateauCount > Constants.ArmConstants.PLATEAU_THRESHOLD);
    }

}