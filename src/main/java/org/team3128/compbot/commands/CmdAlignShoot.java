package org.team3128.compbot.commands;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Pipeline;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightData;
import org.team3128.common.hardware.limelight.LimelightKey;
import org.team3128.common.hardware.limelight.StreamMode;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.units.Angle;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.*;
import org.team3128.compbot.subsystems.Hopper.ActionState;

import com.kauailabs.navx.frc.AHRS;


import org.team3128.compbot.commands.*;

public class CmdAlignShoot extends Command {
    FalconDrive drive;
    Shooter shooter;
    Hopper hopper;
    Arm arm;
    boolean gotDistance = false;

    AHRS ahrs;

    Limelight limelight;

    double decelerationStartDistance, decelerationEndDistance;
    DriveCommandRunning cmdRunning;

    private PIDConstants visionPID;

    private double goalHorizontalOffset;

    private double currentHorizontalOffset;

    private double currentError, previousError;
    private double currentTime, previousTime;

    private double feedbackPower;

    private double leftPower, rightPower;

    private double desiredRPM;
    private double effective_distance;

    private StateTracker stateTracker = StateTracker.getInstance();

    private Command hopperShoot, organize;

    int targetFoundCount;
    int plateauReachedCount;

    int numBallsShot;
    int numBallsToShoot;

    private enum HorizontalOffsetFeedbackDriveState {
        SEARCHING, FEEDBACK; // , BLIND;
    }

    private HorizontalOffsetFeedbackDriveState aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

    public CmdAlignShoot(FalconDrive drive, Shooter shooter, Arm arm, Hopper hopper, AHRS ahrs, Limelight limelight,
            DriveCommandRunning cmdRunning, double goalHorizontalOffset, int numBallsToShoot) {
        this.drive = drive;
        this.shooter = shooter;
        this.arm = arm;
        this.hopper = hopper;
        this.ahrs = ahrs;
        this.limelight = limelight;
        this.visionPID = Constants.VisionConstants.VISION_PID;

        this.cmdRunning = cmdRunning;

        this.goalHorizontalOffset = goalHorizontalOffset;

        this.numBallsToShoot = numBallsToShoot;
    }

    @Override
    protected void initialize() {
        limelight.setLEDMode(LEDMode.ON);
        cmdRunning.isRunning = false;
        arm.setState(stateTracker.getState().targetArmState);
        shooter.setState(stateTracker.getState().targetShooterState);
        hopper.setAction(Hopper.ActionState.SHOOTING);
        Log.info("CmdAlignShoot", "initialized limelight, aren't I cool!");
    }

    @Override
    protected void execute() {
        switch (aimState) {
            case SEARCHING:
                NarwhalDashboard.put("align_status", "searching");
                if (limelight.hasValidTarget()) {
                    targetFoundCount += 1;
                } else {
                    targetFoundCount = 0;
                }

                if (targetFoundCount > 5) {
                    Log.info("CmdAlignShoot", "Target found.");
                    Log.info("CmdAlignShoot", "Switching to FEEDBACK...");
                    LimelightData initData = limelight.getValues(Constants.VisionConstants.SAMPLE_RATE);

                    // double currLLAngle = arm.getAngle() + Constants.ArmConstants.LIMELIGHT_ARM_ANGLE + Constants.VisionConstants.BOTTOM_LIMELIGHT_ANGLE;

                    //                                                                                                     // add
                    // double limelight_height = Constants.VisionConstants.PIVOT_HEIGHT + (Constants.ArmConstants.LIMELIGHT_ARM_LENGTH * RobotMath.sin(currLLAngle));

                    // double distance = (Constants.GameConstants.SHOOTER_TARGET_HEIGHT - limelight_height)
                    //         / (RobotMath.tan(initData.ty()));

                    // effective_distance = distance / RobotMath.cos(initData.tx());

                    // SmartDashboard.putNumber("effective_distance", effective_distance);
                    // SmartDashboard.putNumber("curLLANgle", currLLAngle);
                    // SmartDashboard.putNumber("liemlight_height", limelight_height);
                    SmartDashboard.putNumber("ty", initData.ty());

                    desiredRPM = shooter.getRPMFromDistance();

                    shooter.setSetpoint(desiredRPM);

                    currentHorizontalOffset = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    previousTime = RobotController.getFPGATime();
                    previousError = goalHorizontalOffset - currentHorizontalOffset;

                    cmdRunning.isRunning = true;

                    aimState = HorizontalOffsetFeedbackDriveState.FEEDBACK;
                }

                break;

            case FEEDBACK:
                NarwhalDashboard.put("align_status", "feedback");
                cmdRunning.isRunning = false;
                if (!limelight.hasValidTarget()) {
                    Log.info("CmdAlignShoot", "No valid target.");
                    Log.info("CmdAlignShoot", "Returning to SEARCHING...");

                    aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

                } else {

                    if (!gotDistance) {
                        LimelightData initData = limelight.getValues(Constants.VisionConstants.SAMPLE_RATE);

                        // double currLLAngle = arm.getAngle() + Constants.ArmConstants.LIMELIGHT_ARM_ANGLE + Constants.VisionConstants.BOTTOM_LIMELIGHT_ANGLE;

                        // double limelight_height = Constants.VisionConstants.PIVOT_HEIGHT + (Constants.ArmConstants.LIMELIGHT_ARM_LENGTH * RobotMath.sin(currLLAngle));

                        // double distance = (Constants.GameConstants.SHOOTER_TARGET_HEIGHT - limelight_height)
                        //         / (RobotMath.tan(initData.ty()));

                        // effective_distance = distance / RobotMath.cos(initData.tx());

                        desiredRPM = shooter.getRPMFromDistance();

                        shooter.setSetpoint(desiredRPM);

                        // SmartDashboard.putNumber("effective_distance", effective_distance);
                        // SmartDashboard.putNumber("curLLANgle", currLLAngle);
                        // SmartDashboard.putNumber("liemlight_height", limelight_height);
                        SmartDashboard.putNumber("ty", initData.ty());


                        gotDistance = true;
                    }

                    currentHorizontalOffset = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    currentTime = RobotController.getFPGATime();
                    currentError = goalHorizontalOffset - currentHorizontalOffset;

                    /**
                     * PID feedback loop for the left and right powers based on the horizontal
                     * offset errors.
                     */
                    feedbackPower = 0;

                    feedbackPower += visionPID.kP * currentError;
                    feedbackPower += visionPID.kD * (currentError - previousError) / (currentTime - previousTime);

                    leftPower = RobotMath.clamp(-feedbackPower, -1, 1);
                    rightPower = RobotMath.clamp(feedbackPower, -1, 1);

                    SmartDashboard.putNumber("Shooter Power", leftPower);

                    double leftSpeed = leftPower * Constants.DriveConstants.DRIVE_HIGH_SPEED;
                    double rightSpeed = rightPower * Constants.DriveConstants.DRIVE_HIGH_SPEED;
                    
                    drive.setWheelPower(new DriveSignal(leftPower, rightPower));

                    previousTime = currentTime;
                    previousError = currentError;
                }

                break;

            // case BLIND:
            // NarwhalDashboard.put("align_status", "blind");

            // currentAngle = gyro.getAngle();

            // currentTime = RobotController.getFPGATime() / 1000000.0;
            // currentError = -currentAngle;

            // /**
            // * PID feedback loop for the left and right powers based on the gyro angle
            // */
            // feedbackPower = 0;

            // feedbackPower += blindPID.kP * currentError;
            // feedbackPower += blindPID.kD * (currentError - previousError) / (currentTime
            // - previousTime);

            // rightPower = RobotMath.clamp(blindPID.kF - feedbackPower, -1, 1);
            // leftPower = RobotMath.clamp(blindPID.kF + feedbackPower, -1, 1);

            // Log.info("CmdAlignShoot", "L: " + leftPower + "; R: " + rightPower);

            // drive.tankDrive(leftPower, rightPower);

            // previousTime = currentTime;
            // previousError = currentError;
            // Log.info("CmdAlignShoot", "Error:" + currentError);

            // break;
        }

        if ((Math.abs(currentError) < Constants.VisionConstants.TX_THRESHOLD) && shooter.isReady()) {
            // Log.info("CmdAlignShoot", "Trying to shoot ball");
            hopper.shoot();
        } else {
            hopper.unShoot();
            // Log.info("CmdAlignShoot", "no longer ready to shoot ball");
            // Log.info("CmdAlignShoot", "" + shooter.isReady());
            // Log.info("CmdAlignShoot", "" + Math.abs(currentError));
        }
    }

    @Override
    protected boolean isFinished() {
        if (hopper.isEmpty() || numBallsShot >= numBallsToShoot) {
        return true;
        } else {
        return false;
        }
    }

    @Override
    protected void end() {
        limelight.setLEDMode(LEDMode.OFF);
        drive.stopMovement();
        shooter.setSetpoint(0);
        cmdRunning.isRunning = true;

        Log.info("CmdAlignShoot", "Command Finished.");
        hopper.setAction(Hopper.ActionState.ORGANIZING);
    }

    @Override
    protected void interrupted() {
        end();
    }
}