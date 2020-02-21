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
    static double setpoint = 0; // rotations per minute
    double current = 0;
    double error = 0;
    public double output = 0;
    double accumulator = 0;
    double prevError = 0;

    int plateauCount = 0;

    private Shooter() {
        configMotors();
        configEncoders();
        setSetpoint(0);
    }

    private void configMotors() {
        LEFT_SHOOTER = new LazyCANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_LEFT_ID, MotorType.kBrushless);
        RIGHT_SHOOTER = new LazyCANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_RIGHT_ID, MotorType.kBrushless);
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
        return Constants.ShooterConstants.SHOOTER_GEARING * SHOOTER_ENCODER.getVelocity();
    }

    public void setSetpoint(double passedSetpoint) {
        setpoint = passedSetpoint;
        Log.info("Shooter", "Set setpoint to" + String.valueOf(setpoint));
    }

    @Override
    public void update() {
        current = getRPM();
        // Log.info("Shooter", "Shooter RPM is " + String.valueOf(current));
        error = setpoint - current;
        accumulator += error * Constants.MechanismConstants.DT;
        if (accumulator > Constants.ShooterConstants.SHOOTER_SATURATION_LIMIT) {
            accumulator = Constants.ShooterConstants.SHOOTER_SATURATION_LIMIT;
        } else if (accumulator < -Constants.ShooterConstants.SHOOTER_SATURATION_LIMIT) {
            accumulator = -Constants.ShooterConstants.SHOOTER_SATURATION_LIMIT;
        }
        double kP_term = Constants.ShooterConstants.SHOOTER_PID.kP * error;
        double kI_term = Constants.ShooterConstants.SHOOTER_PID.kI * accumulator;
        double kD_term = Constants.ShooterConstants.SHOOTER_PID.kD * (error - prevError)
                / Constants.MechanismConstants.DT;

        double voltage_output = shooterFeedForward(setpoint) + kP_term + kI_term + kD_term;
        double voltage = RobotController.getBatteryVoltage(); // TODO: investigate bus voltage

        output = voltage_output / voltage;

        prevError = error;

        if (Math.abs(error) <= Constants.ShooterConstants.RPM_THRESHOLD) {
            plateauCount++;
        } else {
            plateauCount = 0;
        }

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

    public double shooterFeedForward(double desiredSetpoint) {
        double ff = (0.00211 * desiredSetpoint) + 0.051; // 0.041
        return ff;
    }

    public double getRPMFromDistance(double distance) {
        return 5300;
        // TODO: relationship between RPM and distance
    }

    public boolean isReady() {
        return (plateauCount > Constants.ShooterConstants.PLATEAU_COUNT);
    }
}