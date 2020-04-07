package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Constants;
import edu.wpi.first.wpilibj.command.Command;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.common.utility.Log;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class CmdSetIndexer extends Command {
    
    Hopper hopper;  
    double power;
    public CmdSetIndexer(Hopper hopper, double power) {
        this.hopper = hopper;
        this.power = power;
    }
    
    @Override
    protected void initialize() {
        hopper.HOPPER_FEEDER_MOTOR.set(ControlMode.PercentOutput, power);
    }
    
    @Override
    protected boolean isFinished() {
        return true;
    }

}