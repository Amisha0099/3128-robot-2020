package org.team3128.gromit.autonomous;

import org.team3128.common.utility.enums.Direction;
import org.team3128.common.utility.units.Length;
import org.team3128.gromit.mechanisms.OptimusPrime;
import org.team3128.gromit.mechanisms.OptimusPrime.RobotState;
import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdInitAuto extends CommandGroup {
    public CmdInitAuto() {
        SRXTankDrive drive = SRXTankDrive.getInstance();
        OptimusPrime op = OptimusPrime.getInstance();
        //addSequential(op.new CmdSetState(RobotState.STARTING));
		//addSequential(op.new CmdSetState(RobotState.INIT));
    }
}