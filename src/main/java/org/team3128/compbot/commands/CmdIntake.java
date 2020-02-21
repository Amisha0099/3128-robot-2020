// authors:
//   The Cult of Mason

package org.team3128.compbot.commands;

import java.lang.invoke.WrongMethodTypeException;
import java.security.GuardedObject;
import java.util.Arrays;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Arm;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.compbot.subsystems.Hopper.HopperState;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.command.Command;

import org.team3128.common.utility.Log;

public class CmdIntake extends Command {
    Hopper hopper;
    Arm arm;
    int armCount;
    boolean empty = true;
    //boolean empty1 = true;
    boolean broken = false;
    double currentTime;
    double brokenTime;
    double startPos;
    int position = 0;
    boolean isGoing = false;
    boolean[] updatedArray = new boolean[Constants.HopperConstants.CAPACITY];

    boolean inFocus = false;
    int focusPos = 0;

    public CmdIntake(Hopper hopper, Arm arm) {
        this.hopper = hopper;
        this.arm = arm;
    }

    @Override
    protected void initialize() {
        // nothing here
        arm.setState(ArmState.INTAKE);
        if (hopper.getNumBalls() == Constants.HopperConstants.CAPACITY) {
            armCount = Constants.HopperConstants.CAPACITY - 1;
        } else {
            armCount = hopper.getNumBalls();
        }
    }

    @Override
    protected void execute() {
        //updateCount();
        updatedArray = hopper.getBallArray();
        if (!hopper.isFull()) {
            hopper.INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_ON_VALUE);
            //hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
        } else {
            hopper.INTAKE_MOTOR.set(0);
            //hopper.setMotorPowers(0, 0, 0);
        }

        if (hopper.SENSOR_1.get()) {
            hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
            empty = false;
        } else if(!empty) {
            empty = true;
            armCount++;
            switch (armCount) {
                case 1:
                    updatedArray = new boolean[] {false, false, false, true, false};
                    break;
                case 2:
                    updatedArray = new boolean[] {false, false, true, true, false};
                    break;
                case 3:
                    updatedArray = new boolean[] {false, true, true, true, false};
                    break;
                case 4:
                    updatedArray = new boolean[] {true, true, true, true, false};
                    break;
            }
            startPos = hopper.CORNER_ENCODER.getPosition();
            hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
            isGoing = true;
        }
        if (isGoing && hopper.CORNER_ENCODER.getPosition() - Constants.HopperConstants.BALL_SPACING[hopper.getNumBalls() - 1] < startPos) {
            hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
            isGoing = false;
        }

        hopper.updateBallArray(updatedArray);
    }

    @Override
    protected boolean isFinished() {
        if (hopper.getNumBalls() == Constants.HopperConstants.CAPACITY) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    protected void end() {
        // handle what happens when the command is terminated
        brokenTime = Timer.getFPGATimestamp();
        currentTime = brokenTime;
        hopper.INTAKE_MOTOR.set(0);
        while(brokenTime - 250 < currentTime) {
            if (hopper.SENSOR_1.get()) {
                hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                empty = false;
            } else if(!empty) {
                empty = true;
                armCount++;
                switch (armCount) {
                    case 1:
                        updatedArray = new boolean[] {false, false, false, true, false};
                        break;
                    case 2:
                        updatedArray = new boolean[] {false, false, true, true, false};
                        break;
                    case 3:
                        updatedArray = new boolean[] {false, true, true, true, false};
                        break;
                    case 4:
                        updatedArray = new boolean[] {true, true, true, true, false};
                        break;
                }
                startPos = hopper.CORNER_ENCODER.getPosition();
                hopper.setMotorPowers(0, Constants.HopperConstants.BASE_POWER, Constants.HopperConstants.BASE_POWER);
                isGoing = true;
            }
            if (isGoing && hopper.CORNER_ENCODER.getPosition() - Constants.HopperConstants.BALL_SPACING[hopper.getNumBalls() - 1] < startPos) {
                hopper.setMotorPowers(0, 0, Constants.HopperConstants.BASE_POWER);
            }
            currentTime = Timer.getFPGATimestamp();
        }
        hopper.setMotorPowers(0, 0, 0);
    }

    @Override
    protected void interrupted() {
        end(); // check if the balls are in a good position. If not, then shuffle them around
               // to the right position
    }

    // private void updateCount() {
    //     if (hopper.SENSOR_1.get()) {
    //         empty = false;
    //     } else if (!empty) {
    //         armCount++;
    //         empty = true;
    //     }
    // }
}