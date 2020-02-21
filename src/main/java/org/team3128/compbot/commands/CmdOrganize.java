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


public class CmdOrganize extends Command {
    Hopper hopper;
    int topCount;
    int middleCount;
    int totalCount;
    boolean behindFeeder = false;
    boolean empty0 = false;
    boolean empty1 = false;
    boolean empty2 = false;
    boolean broken = false;
    boolean firstBallPast = false;
    int position = 0;
    double brokenTime = 0;
    double currentTime = 0;
    boolean[] updatedArray = new boolean[Constants.HopperConstants.CAPACITY];

    boolean inFocus = false;
    int focusPos = 0;

    public CmdOrganize(Hopper hopper) {
        this.hopper = hopper;
    }

    @Override
    protected void initialize() {
        // nothing here
        if (hopper.getNumBalls() != 3) {
            Log.info("CmdOrganize", "aaaaaaaaahhhhhhhhh you caaaaaahhhhllled cmdorgaahhnnize when you shouldn't haaahhhve");
            //Log.info("CmdOrganize", "" + 1 / 0);
        }
        hopper.setMotorPowers(0, -Constants.HopperConstants.BASE_POWER, -Constants.HopperConstants.BASE_POWER);
    }

    @Override
    protected void execute() {
        // if (hopper.SENSOR_2.get() && !empty2) {
        //     empty2 = true;
        // }
        // if (!hopper.SENSOR_2.get() && empty2) {
        //     behindFeeder = true;
        // }
    }

    @Override
    protected boolean isFinished() {
        //return hopper.SENSOR_2.get();// && behindFeeder;
        return true; //TODO: this is bs now lol
    }

    @Override
    protected void end() {
        brokenTime = Timer.getFPGATimestamp();
        currentTime = brokenTime;
        while(currentTime - 100 < brokenTime) { // TODO: tune times
            currentTime = Timer.getFPGATimestamp(); // TODO: maybe add timeouts
        }
        hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, 0);
        while(!hopper.SENSOR_0.get() || !firstBallPast) {
            if (hopper.SENSOR_0.get()) {
                firstBallPast = true;
            }
        }
        hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
        while(!hopper.SENSOR_1.get()) {
        }
        hopper.setMotorPowers(0, 0, 0);
        //hopper.setBallOrder(hopper.HopperState.POS_3.hopperState);
        hopper.setBallOrder(new boolean[] {true, true, false, true, false});
    }

    @Override
    protected void interrupted() {
        end();
    }

}