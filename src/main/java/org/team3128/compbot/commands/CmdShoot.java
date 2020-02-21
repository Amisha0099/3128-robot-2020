//authors:
//   The Cult of Mason
//   Tyler

package org.team3128.compbot.commands;

import java.lang.invoke.WrongMethodTypeException;
import java.security.GuardedObject;
import java.util.Arrays;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.compbot.subsystems.Hopper.HopperState;

import edu.wpi.first.wpilibj.command.Command;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.Timer;


public class CmdShoot extends Command {
    Hopper hopper;
    int topCount;
    int middleCount;
    int totalCount;
    boolean empty0 = false;
    boolean empty1 = false;
    boolean empty2 = false;
    boolean broken = false;
    int position = 0;
    double brokenTime = 0;
    double currentTime = 0;
    boolean[] updatedArray = new boolean[Constants.HopperConstants.CAPACITY];

    boolean inFocus = false;
    int focusPos = 0;

    public CmdShoot(Hopper hopper) {
        this.hopper = hopper;
    }

    @Override
    protected void initialize() {
        // nothing here
        switch(hopper.getNumBalls()) {
            case 0:
                topCount = 0;
                middleCount = 0;
                totalCount = 0;
                break;
            case 1:
                topCount = 1;
                middleCount = 0;
                totalCount = 1;
                break;
            case 2:
                topCount = 2;
                middleCount = 0;
                totalCount = 2;
                break;
            case 3:
                topCount = 2;
                middleCount = 1;
                totalCount = 3;
                break;
            case 4:
                topCount = 2;
                middleCount = 2;
                totalCount = 4;
                break;
            case 5:
                topCount = 2;
                middleCount = 2;
                totalCount = 5;
                break;
        }
        hopper.setMotorPowers(Constants.HopperConstants.GATEKEEPER_POWER, 0, 0);
    }

    @Override
    protected void execute() {
        // if (hopper.getNumBalls() == Constants.HopperConstants.CAPACITY) {
        //     hopper.setMotorPowers(Constants.HopperConstants.GATEKEEPER_POWER, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
        // } else if (hopper.getNumBalls() == 0) {
        //     hopper.setMotorPowers(0, 0, 0);
        //     Log.info("CmdShoot", "hopper is ready is broken there are 0 balls in the shooter");
        // } else {
        //     hopper.setMotorPowers(Constants.HopperConstants.GATEKEEPER_POWER, Constants.HopperConstants.BASE_POWER, 0);
        // }
    }

    @Override
    protected boolean isFinished() {
        return Timer.getFPGATimestamp() > 1000; //TODO: tune times
    }

    @Override
    protected void end() {
        hopper.setBallOrder(hopper.shift(hopper.getBallArray()));
        if (hopper.getNumBalls() > 1) {
            while(!hopper.SENSOR_0.get() || empty0 == false) {
                hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, 0);
                if(!hopper.SENSOR_0.get()) {
                    empty0 = true;
                }
            }
        } else {
            brokenTime = Timer.getFPGATimestamp();
            currentTime = brokenTime;
            while (currentTime - 500 > brokenTime) {
                hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, 0);
                currentTime = Timer.getFPGATimestamp();
            }
        }
        hopper.setMotorPowers(0, 0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }
}