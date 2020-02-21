package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Constants;
import edu.wpi.first.wpilibj.command.Command;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Arm;

public class CmdSetArm extends Command {
    
    Arm arm;
    ArmState armState;
    
    public CmdSetArm(Arm arm, ArmState armState) {
        this.arm = arm;
        this.armState = armState;
    }
    
    @Override
    protected void initialize() {
        arm.setState(armState);
    }
    
    @Override
    protected void execute() {
        
    }
    
    @Override
    protected boolean isFinished() {
        if (arm.isReady()){
            return true;
        } else {
            return false;
        }
    }

}