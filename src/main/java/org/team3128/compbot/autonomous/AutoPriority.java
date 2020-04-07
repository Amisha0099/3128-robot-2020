package org.team3128.compbot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.drive.Drive;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.compbot.autonomous.*;
import org.team3128.compbot.commands.*;
import org.team3128.compbot.subsystems.Constants.VisionConstants;
import org.team3128.compbot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;

public class AutoPriority extends CommandGroup {

    public AutoPriority(FalconDrive drive, Shooter shooter, Arm arm, Hopper hopper, AHRS ahrs, Limelight limelight, DriveCommandRunning cmdRunning, double timeoutMs) {       
        addSequential(new CmdAlignShoot(drive, shooter, arm, hopper, ahrs, limelight, cmdRunning, Constants.VisionConstants.TX_OFFSET, 3));
        addSequential(new CmdAutoTrajectory(drive, 120, 0.5, 10000, 
            new Pose2D(0, 0, Rotation2D.fromDegrees(0)),
            new Pose2D(194 * Constants.MechanismConstants.inchesToMeters, 27 * Constants.MechanismConstants.inchesToMeters, Rotation2D.fromDegrees(180)))); // 194.63 inches length and 27.75 inches width
        // for (int i = 0; i < 3; i++) { // run three times because we are picking up three balls
        //     addSequential(new CmdBallIntake(ahrs, limelight, hopper, arm, cmdRunning, Constants.VisionConstants.BALL_PID, Constants.VisionConstants.BLIND_BALL_PID, 0.472441 * Constants.MechanismConstants.inchesToMeters, Constants.VisionConstants.TX_OFFSET));
        // }
        addSequential(new CmdAutoTrajectory(drive, 120, 0.5, 10000, 
            new Pose2D(0, 0, Rotation2D.fromDegrees(0)),
            new Pose2D(0, 0, Rotation2D.fromDegrees(180)))); // TODO: check if this rotate in place
        addSequential(new CmdAlignShoot(drive, shooter, arm, hopper, ahrs, limelight, cmdRunning, Constants.VisionConstants.TX_OFFSET, 3));
    }
}