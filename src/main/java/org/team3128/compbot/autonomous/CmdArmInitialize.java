package org.team3128.compbot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.team3128.compbot.autonomous.CmdSetArm;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Arm;
import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.common.utility.Log;


public class CmdArmInitialize extends CommandGroup {
    
    public CmdArmInitialize(Arm arm, Hopper hopper) {
        addSequential(new CmdSetIntake(hopper, Constants.IntakeConstants.INTAKE_MOTOR_REVERSE_VALUE));
        addSequential(new CmdSetArm(arm, ArmState.STARTING_DOWN, 1000)); 
        addSequential(new CmdSetArm(arm, ArmState.STARTING, 1000));
        addSequential(new CmdSetIntake(hopper, Constants.IntakeConstants.INTAKE_MOTOR_OFF_VALUE));
    }

}