package org.team3128.compbot.subsystems;

import org.team3128.common.utility.Log;
import org.team3128.common.utility.test_suite.CanDevices;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class Shooter extends Threaded {

    public static final Shooter instance = new Shooter();
    public static LazyCANSparkMax LEFT_SHOOTER;
    public static LazyCANSparkMax RIGHT_SHOOTER;
    public static CANEncoder SHOOTER_ENCODER;

    public static boolean DEBUG = true;
    public static int setpoint = 0; // rotations per minute
    double setpoint;
    double current = 0;
    double error = 0;
    double output = 0;
    double accumulator = 0;
    double prevError = 0;

    private Shooter() {
        configMotors();
        configEncoders();
        startVoltage = RobotController.getBatteryVoltage();
    }

    private void configMotors() {
        LEFT_SHOOTER = new LazyCANSparkMax(Constants.SHOOTER_MOTOR_LEFT_ID, MotorType.kBrushless);
        RIGHT_SHOOTER = new LazyCANSparkMax(Constants.SHOOTER_MOTOR_RIGHT_ID, MotorType.kBrushless);
        if (DEBUG) {
            Log.info("Shooter", "Config motors");
        }
    }

    private void configEncoders() {
        SHOOTER_ENCODER = LEFT_SHOOTER.getEncoder();
        if (DEBUG) {
            Log.info("Shooter", "Config encoders");
        }
    }

    public static Shooter getInstance() {
        return instance;
    }

    public static double getRPM() {
        return SHOOTER_ENCODER.getVelocity();
    }

    public static void setSetpoint(int passedSetpoint) {
        setpoint = passedSetpoint;
    }

    @Override
    public void update() {
        current = getRPM();
        error = setpoint - current;
        accumulator += error * Constants.DT;
        if (accumulator > Constants.SHOOTER_SATURATION_LIMIT) {
            accumulator = Constants.SHOOTER_SATURATION_LIMIT;
        } else if (accumulator < -Constants.SHOOTER_SATURATION_LIMIT) {
            accumulator = -Constants.SHOOTER_SATURATION_LIMIT;
        }
        double kP_term = Constants.kP_SHOOTER * error;
        double kI_term = Constants.kI_SHOOTER * accumulator;
        double kD_term = Constants.kD_SHOOTER * (error - prevError) / Constants.DT;

        double voltage_output = shooterFeedForward(setpoint) + kP_term + kI_term + kD_term;
        double voltage = RobotController.getBatteryVoltage();

        output = voltage_output / voltage;

        prevError = error;
        if (output > 1) {
            Log.info("SHOOTER",
                    "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this ");
            output = 1;
        } else if (output < -1) {
            Log.info("SHOOTER",
                    "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this ");
            output = -1;
        }

        LEFT_SHOOTER.set(output);
        RIGHT_SHOOTER.set(-output);

    }

    private double shooterFeedForward(int setpoint2) {
        return 0; // TODO: add feedforward implementation for arm control
    }
}