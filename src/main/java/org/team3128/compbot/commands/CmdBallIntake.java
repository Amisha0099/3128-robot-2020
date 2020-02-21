package org.team3128.compbot.commands;

import java.lang.invoke.WrongMethodTypeException;
import java.security.GuardedObject;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.command.Command;

import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.compbot.subsystems.Arm;
import org.team3128.compbot.subsystems.Hopper.HopperState;
import org.team3128.common.utility.Log;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.compbot.commands.*;

public class CmdBallIntake extends CommandGroup {
    
    public CmdBallIntake(Gyro gyro, Limelight ballLimelight, Hopper hopper, Arm arm, DriveCommandRunning cmdRunning, PIDConstants visionPID, PIDConstants blindPID, double targetHeight, double offset) {
        
        /*addParallel(//new CmdRunInParallel(
            new CmdBallPursuit(
                gyro, ballLimelight, cmdRunning, targetHeight * Length.cm,
                visionPID, offset, 2.5 * Length.ft, 0.6666666666666666666666 * Length.ft,
                blindPID, 42 * Angle.DEGREES)//,
                //new CmdStreamUpdate(bottomLimelight, topLimelight, useBottom)
        );
        addParallel(
            new CmdIntake(hopper)
        );
        */

        addSequential(new CmdIntake(hopper, arm));
    }

}
        