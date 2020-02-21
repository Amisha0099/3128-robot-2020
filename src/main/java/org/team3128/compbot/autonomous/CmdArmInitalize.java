package org.team3128.compbot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.team3128.compbot.autonomous.CmdSetArm;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Arm;

public class CmdArmInitalize extends CommandGroup {
    
    public CmdArmInitalize(Arm arm) {
        addSequential(new CmdSetArm(arm, ArmState.STARTING_DOWN)); 
        addSequential(new CmdSetArm(arm, ArmState.STARTING)); 
    }

}