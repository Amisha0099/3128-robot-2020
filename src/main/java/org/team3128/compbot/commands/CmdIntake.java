package org.team3128.compbot.commands;

import java.lang.invoke.WrongMethodTypeException;
import java.security.GuardedObject;
import java.util.Arrays;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.compbot.subsystems.Arm;

import edu.wpi.first.wpilibj.command.Command;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.Timer;


public class CmdIntake extends Command {
    Hopper hopper;
    Arm arm;
    int startingBallCount;
    
    public CmdIntake(Hopper hopper, Arm arm) {
        this.hopper = hopper;
        this.arm = arm;
    }

    @Override
    protected void initialize() {
        startingBallCount = hopper.getBallCount();
    }

    @Override
    protected void execute() {
        if (arm.ARM_STATE == Arm.ArmState.INTAKE) {
                hopper.setAction(Hopper.ActionState.INTAKING);
        } else {
                arm.setState(Arm.ArmState.INTAKE);
        }
    }

    @Override
    protected boolean isFinished() {
        if ((hopper.getBallCount() - startingBallCount) == 1) { // this logic is flawed if there are already three balls in the hopper
                                                                // but, hopefully, this will only be run when the hopper is empty with our autos
            return true;
        }
        return false;
    }
}