package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Constants;
import edu.wpi.first.wpilibj.command.Command;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.common.utility.Log;

public class CmdSetIntake extends Command {
    
    Hopper hopper;  
    double power;
    public CmdSetIntake(Hopper hopper, double power) {
        this.hopper = hopper;
        this.power = power;
    }
    
    @Override
    protected void initialize() {
        hopper.INTAKE_MOTOR.set(power);
    }
    
    @Override
    protected boolean isFinished() {
        return true;
    }

}