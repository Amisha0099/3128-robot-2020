/** @author:Jude, Daniel, Tyler, Jenny, Priyanka */

package org.team3128.compbot.commands;

import java.lang.invoke.WrongMethodTypeException;
import java.security.GuardedObject;
import java.util.Arrays;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Hopper;

import edu.wpi.first.wpilibj.command.Command;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.Timer;


public class CmdEjectBalls extends Command {
    Hopper hopper;
    int adjustBallCount;
    double currentTime;
    double startTime;
    int timeCounter = 0;
    

    public CmdEjectBalls(Hopper hopper) {
        this.hopper = hopper;
    }

    @Override
    protected void initialize() {
        //nothing here
        startTime = Timer.getFPGATimestamp();
        hopper.setMotorPowers(-Constants.HopperConstants.GATEKEEPER_POWER, -Constants.HopperConstants.BASE_POWER, -Constants.HopperConstants.BASE_POWER);
        hopper.INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_REVERSE_VALUE);
    }

    @Override
    protected void execute() {
        timeCounter++;
        if (timeCounter == 5) {
            currentTime = Timer.getFPGATimestamp();
            timeCounter = 0;
        }
    }

    @Override
    protected boolean isFinished() {
        return ((currentTime - startTime) >= 10000); //if the adjusted ball count is 0 or 10 seconds has gone by, return true
    }

    @Override
    protected void end() {
        hopper.setMotorPowers(0, 0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }
}