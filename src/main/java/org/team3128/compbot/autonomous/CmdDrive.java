package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Constants;
import edu.wpi.first.wpilibj.command.Command;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.FalconDrive;
import org.team3128.common.utility.Log;
import edu.wpi.first.wpilibj.Timer;
import org.team3128.common.drive.DriveSignal;

public class CmdDrive extends Command {
    
    FalconDrive drive;
    double timeoutMs, startTime;
    
    public CmdDrive(FalconDrive drive, double timeoutMs) {
        this.drive = drive;
        this.timeoutMs = timeoutMs;
    }
    
    @Override
    protected void initialize() {
        drive.setWheelPower(new DriveSignal(0.3, 0.3));
        startTime = Timer.getFPGATimestamp();
    }
    
    @Override
    protected boolean isFinished() {
        if (((Timer.getFPGATimestamp() - startTime) >= timeoutMs)){
            return true;
        } else {
            return false;
        }
    }

    @Override
    protected void end() {
        drive.setWheelPower(new DriveSignal(0, 0));
    }

    @Override
    protected void interrupted() {
        end();
    }



}