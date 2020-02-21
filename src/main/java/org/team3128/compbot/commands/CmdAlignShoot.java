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

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.*;
import org.team3128.compbot.subsystems.Hopper.ActionState;
import org.team3128.compbot.commands.*;

public class CmdAlignShoot extends Command {
    FalconDrive drive;
    Shooter shooter;
    Hopper hopper;
    Arm arm;
    boolean gotDistance = false;

    Gyro gyro;

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

    private Command hopperShoot, organize;

    int targetFoundCount;
    int plateauReachedCount;

    int numBallsShot;
    int numBallsToShoot;

    private enum HorizontalOffsetFeedbackDriveState {
        SEARCHING, FEEDBACK; // , BLIND;
    }

    private HorizontalOffsetFeedbackDriveState aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

    public CmdAlignShoot(FalconDrive drive, Shooter shooter, Arm arm, Hopper hopper, Gyro gyro, Limelight limelight,
            DriveCommandRunning cmdRunning, double goalHorizontalOffset, int numBallsToShoot) {
        this.drive = drive;
        this.shooter = shooter;
        this.arm = arm;
        this.hopper = hopper;
        this.gyro = gyro;
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

                    double currLLAngle = arm.getAngle() + Constants.ArmConstants.LIMELIGHT_ARM_ANGLE;

                    double limelight_height = Constants.ArmConstants.LIMELIGHT_ARM_LENGTH * RobotMath.sin(currLLAngle); // TODO:
                                                                                                                        // add
                                                                                                                        // limelight
                                                                                                                        // height
                                                                                                                        // to
                                                                                                                        // this
                                                                                                                        // (when
                                                                                                                        // stowed)

                    double distance = (Constants.GameConstants.SHOOTER_TARGET_HEIGHT - limelight_height)
                            / (RobotMath.tan(initData.ty()));

                    effective_distance = distance / RobotMath.cos(initData.tx());

                    desiredRPM = shooter.getRPMFromDistance(effective_distance);

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
                if (!limelight.hasValidTarget()) {
                    Log.info("CmdAlignShoot", "No valid target.");
                    Log.info("CmdAlignShoot", "Returning to SEARCHING...");

                    aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

                    cmdRunning.isRunning = false;
                } else {

                    if (!gotDistance) {
                        LimelightData initData = limelight.getValues(Constants.VisionConstants.SAMPLE_RATE);

                        double currLLAngle = arm.getAngle() + Constants.ArmConstants.LIMELIGHT_ARM_ANGLE;

                        double limelight_height = Constants.ArmConstants.LIMELIGHT_ARM_LENGTH
                                * RobotMath.sin(currLLAngle); // TODO:
                                                              // add
                                                              // limelight
                                                              // height
                                                              // to
                                                              // this
                                                              // (when
                                                              // stowed)

                        double distance = (Constants.GameConstants.SHOOTER_TARGET_HEIGHT - limelight_height)
                                / (RobotMath.tan(initData.ty()));

                        effective_distance = distance / RobotMath.cos(initData.tx());

                        desiredRPM = shooter.getRPMFromDistance(effective_distance);

                        shooter.setSetpoint(desiredRPM);

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

        if ((currentError < Constants.VisionConstants.TX_THRESHOLD) && shooter.isReady()) {
            // hopperShoot = new CmdShoot(hopper);
            // hopperShoot.start();
            hopper.shoot();
            numBallsShot++;
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

        // NarwhalDashboard.put("align_status", "blind");

        Log.info("CmdAlignShoot", "Command Finished.");
        hopper.setAction(Hopper.ActionState.ORGANIZING);
    }

    @Override
    protected void interrupted() {
        end();
        // drive.stopMovement();
        // limelight.setLEDMode(LEDMode.OFF);

        // // NarwhalDashboard.put("align_status", "blind");

        // cmdRunning.isRunning = false;

        // Log.info("CmdAlignShoot", "Command Finished.");
    }
}