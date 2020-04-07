package org.team3128.compbot.calibration;

import java.lang.invoke.WrongMethodTypeException;
import java.security.GuardedObject;
import java.util.Arrays;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Arm;

import edu.wpi.first.wpilibj.command.Command;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonFX;

public class CmdArmFF extends Command {

    Arm arm;

    double currentAngle = 0;
    double error = 0;
    double output = 0;
    double accumulator = 0;
    double prevError = 0;
    double previousAngle;
    double sumAngle;

    public static int setpoint = 0;
    public static int increment = 5;
    public static int counter = 0;
    public static double sumOutput, sumBatteryVoltage, sumRPM, sumBusVoltage, startVoltage, currentTime, pastTime;

    int plateauCount = 0;

    public CmdArmFF(Arm arm) {
        this.arm = arm;
    }

    @Override
    protected void initialize() {
        previousAngle = arm.getAngle();
        pastTime = Timer.getFPGATimestamp();
    }

    @Override
    protected void execute() {

        currentAngle = arm.getAngle();

        error = setpoint - currentAngle;
        accumulator += error * Constants.MechanismConstants.DT;
        if (accumulator > Constants.ArmConstants.ARM_SATURATION_LIMIT) {
            accumulator = Constants.ArmConstants.ARM_SATURATION_LIMIT;
        } else if (accumulator < -Constants.ArmConstants.ARM_SATURATION_LIMIT) {
            accumulator = -Constants.ArmConstants.ARM_SATURATION_LIMIT;
        }

        double kP_term = Constants.ArmConstants.ARM_PID.kP * error;
        double kI_term = Constants.ArmConstants.ARM_PID.kI * accumulator;
        double kD_term = Constants.ArmConstants.ARM_PID.kD * (error - prevError) / Constants.MechanismConstants.DT;

        double voltage_output = arm.armFeedForward(setpoint) + kP_term + kI_term + kD_term;
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

        arm.ARM_MOTOR_LEADER.set(ControlMode.PercentOutput, output);

        currentTime = Timer.getFPGATimestamp();
        currentAngle = arm.getAngle();

        if (Math.abs((previousAngle - currentAngle) / (pastTime - currentTime)) <= 0.001) {
            counter += 1;

            sumOutput += output;
            sumBatteryVoltage += RobotController.getBatteryVoltage();
            sumAngle += currentAngle;
            sumBusVoltage += arm.ARM_MOTOR_LEADER.getMotorOutputVoltage();

            if (counter >= 10) {
                Log.info("Shooter", "Current Setpoint: " + setpoint + " Average Angle: " + (sumAngle / counter)
                        + " Average Voltage Battery: " + (sumBatteryVoltage / counter) + " Average Voltage Bus: "
                        + (sumBusVoltage / counter) + " Average Percent Output: " + (sumOutput / counter));
                setpoint += increment;
                counter = 0;
                sumOutput = 0;
                sumBatteryVoltage = 0;
                sumAngle = 0;
                sumBusVoltage = 0;
            }
        }

        pastTime = currentTime;
        prevError = error;
        previousAngle = currentAngle;
    }

    @Override
    protected boolean isFinished() {
        if (setpoint >= 85) {
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
        arm.ARM_MOTOR_LEADER.set(ControlMode.PercentOutput, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}