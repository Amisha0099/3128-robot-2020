package org.team3128.testbench.subsystems;

import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
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
    public static int setpoint = 250;
    public static int increment = 250;    
    public static int counter = 0;
    public static double sumOutput, sumBatteryVoltage, sumRPM, sumBusVoltage, output, currentError, startVoltage, voltageBattery, voltageMotor, leftOutput, rightPower, pastError, currentTime, pastTime;

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
        pastError = setpoint - getRPM();
        pastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        currentTime = Timer.getFPGATimestamp();
        currentError = setpoint - getRPM();

        output += Constants.K_SHOOTER_P * currentError;
        output += Constants.K_SHOOTER_D * (currentError - pastError) / (currentTime - pastTime);
        output += Constants.K_SHOOTER_FF;

        //output = (Constants.K_SHOOTER_FF + ((Constants.K_SHOOTER_P * currentError) + (Constants.K_SHOOTER_D * (pastError - currentError) / (pastTime - currentTime))));
        output = RobotMath.clamp(output, -1, 1);

        // Log.info("[Shooter]", " " + output);
        voltageBattery = RobotController.getBatteryVoltage();
        voltageMotor = LEFT_SHOOTER.getBusVoltage();

        LEFT_SHOOTER.set(output);
        RIGHT_SHOOTER.set(output);

        if (Math.abs(setpoint - getRPM()) <= 5){
            counter += 1;

            sumOutput += output;
            sumBatteryVoltage += RobotController.getBatteryVoltage(); 
            sumRPM += getRPM();
            sumBusVoltage += LEFT_SHOOTER.getBusVoltage();

            if ((increment) != 0 && (counter >= 400)) {
                Log.info("Shooter", "Current Setpoint: " + setpoint + " Average RPM: " + (sumRPM/counter) + " Average Voltage Battery: " + (sumBatteryVoltage/counter) + " Average Voltage Bus: " + (sumBusVoltage/counter) + " Average Percent Output: " + (sumOutput/counter)); 
                setpoint += increment;
                counter = 0;
                sumOutput = 0;
                sumBatteryVoltage = 0; 
                sumRPM = 0; 
                sumBusVoltage = 0; 
            }
        }
        
        if (setpoint >= 4500){
            Log.info("Shooter", "Finished with automated loop");
            setpoint = 0;
            increment = 0;
        }

        if (DEBUG) {
            // Log.info("Shooter", "Error  is: " + error + ", vel is: " + getRPM() + ", output is: " + output + ", voltage battery is " + (startVoltage - voltageBattery) + ", voltage bus is " + voltageMotor);
        }

        pastError = currentError;
        pastTime = currentTime;
    }
}