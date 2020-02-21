package org.team3128.compbot.commands;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Shooter;

import edu.wpi.first.wpilibj.command.Command;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.RobotController;

public class CmdShooterFF extends Command {

    Shooter shooter;

    double current = 0;
    double error = 0;
    double output = 0;
    double accumulator = 0;
    double prevError = 0;

    public static int setpoint = 250;
    public static int increment = 250;
    public static int counter = 0;
    public static double sumOutput, sumBatteryVoltage, sumRPM, sumBusVoltage, startVoltage, voltageBattery,
            voltageMotor, currentTime, pastTime;

    int plateauCount = 0;

    public CmdShooterFF(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {

        /*
         * current = shooter.getRPM(); error = setpoint - current; accumulator += error
         * * Constants.MechanismConstants.DT; if (accumulator >
         * Constants.ShooterConstants.SHOOTER_SATURATION_LIMIT) { accumulator =
         * Constants.ShooterConstants.SHOOTER_SATURATION_LIMIT; } else if (accumulator <
         * -Constants.ShooterConstants.SHOOTER_SATURATION_LIMIT) { accumulator =
         * -Constants.ShooterConstants.SHOOTER_SATURATION_LIMIT; } double kP_term =
         * Constants.ShooterConstants.SHOOTER_PID.kP * error; double kI_term =
         * Constants.ShooterConstants.SHOOTER_PID.kI * accumulator; double kD_term =
         * Constants.ShooterConstants.SHOOTER_PID.kD * (error - prevError) /
         * Constants.MechanismConstants.DT;
         * 
         * double voltage_output = shooter.shooterFeedForward(setpoint) + kP_term +
         * kI_term + kD_term; double voltage = RobotController.getBatteryVoltage(); //
         * TODO: investigate bus voltage
         * 
         * output = voltage_output / voltage;
         * 
         * prevError = error;
         * 
         * if (Math.abs(error) <= Constants.ShooterConstants.RPM_THRESHOLD) {
         * plateauCount++; } else { plateauCount = 0; }
         * 
         * if (output > 1) { Log.info("SHOOTER",
         * "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this "
         * ); output = 1; } else if (output < -1) { Log.info("SHOOTER",
         * "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this "
         * ); output = -1; } shooter.LEFT_SHOOTER.set(output);
         * shooter.RIGHT_SHOOTER.set(-output);
         */

        shooter.setSetpoint(setpoint);

        voltageBattery = RobotController.getBatteryVoltage();
        voltageMotor = Shooter.LEFT_SHOOTER.getBusVoltage();

        currentTime = Timer.getFPGATimestamp();

        if (Math.abs(setpoint - shooter.getRPM()) <= 10) {
            counter += 1;

            sumOutput += output;
            sumBatteryVoltage += RobotController.getBatteryVoltage();
            sumRPM += shooter.getRPM();
            sumBusVoltage += shooter.LEFT_SHOOTER.getBusVoltage();

            if ((increment) != 0 && (counter >= 400)) {
                Log.info("Shooter", "Current Setpoint: " + setpoint + " Average RPM: " + (sumRPM / counter)
                        + " Average Voltage Battery: " + (sumBatteryVoltage / counter) + " Average Voltage Bus: "
                        + (sumBusVoltage / counter) + " Average Percent Output: " + (sumOutput / counter));
                setpoint += increment;
                counter = 0;
                sumOutput = 0;
                sumBatteryVoltage = 0;
                sumRPM = 0;
                sumBusVoltage = 0;
            }
        }

        pastTime = currentTime;

    }

    @Override
    protected boolean isFinished() {
        if (setpoint >= 4500) {
            Log.info("Shooter", "Finished with automated loop");
            setpoint = 0;
            increment = 0;
            return true;
        } else {
            return false;
        }
    }

    @Override
    protected void end() {
        shooter.LEFT_SHOOTER.set(0);
        shooter.RIGHT_SHOOTER.set(0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}