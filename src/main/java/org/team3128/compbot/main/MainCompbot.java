package org.team3128.compbot.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.team3128.common.generics.RobotConstants;

import com.kauailabs.navx.frc.AHRS;


import org.team3128.common.NarwhalRobot;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.TrajectoryGenerator;
import org.team3128.common.control.trajectory.constraint.TrajectoryConstraint;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.gyroscope.NavX;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.test_suite.CanDevices;
import org.team3128.common.utility.test_suite.ErrorCatcherUtility;
import org.team3128.compbot.commands.*;
import org.team3128.compbot.autonomous.AutoSimple;
import org.team3128.compbot.calibration.*;
import org.team3128.compbot.subsystems.*;
import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.RobotTracker;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Hopper.ActionState;
import org.team3128.compbot.subsystems.StateTracker.RobotState;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.concurrent.*;

import org.team3128.common.generics.ThreadScheduler;

public class MainCompbot extends NarwhalRobot {

    public Command triggerCommand;
    public Command armFFCommand;
    public Command shooterFFCommand;
    public Command ejectBallsCommand;
    private DriveCommandRunning driveCmdRunning;

    public StateTracker stateTracker = StateTracker.getInstance();
    static FalconDrive drive = FalconDrive.getInstance();
    static Hopper hopper = Hopper.getInstance();
    static Arm arm = Arm.getInstance();
    static Shooter shooter = Shooter.getInstance();
    static Climber climber = new Climber();

 
    // RobotTracker robotTracker = RobotTracker.getInstance();

    ExecutorService executor = Executors.newFixedThreadPool(6);
    ThreadScheduler scheduler = new ThreadScheduler();
    Thread auto;

    public Joystick joystickRight, joystickLeft;
    public ListenerManager listenerLeft, listenerRight;
    public AHRS ahrs;
    public static PowerDistributionPanel pdp;

    public NetworkTable table;
    public NetworkTable limelightTable;

    public double startTime = 0;

    public String trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";

    public ArrayList<Pose2D> waypoints = new ArrayList<Pose2D>();
    public Trajectory trajectory;

    public Limelight shooterLimelight, ballLimelight;
    public Limelight[] limelights;
    public boolean inPlace = false;
    public boolean inPlace2 = false;

    public int countBalls = 0;
    public int countBalls2 = 0;
    public static CanDevices leftDriveLeader;
    public static CanDevices leftDriveFollower;
    public static CanDevices rightDriveLeader;
    public static CanDevices rightDriveFollower;
    public static CanDevices PDP;


    public ErrorCatcherUtility errorCatcher;
    public static CanDevices[] CanChain = new CanDevices[42];

    public Command povCommand;
    public Command shooterFF;

    public static void setCanChain() {  
        Constants.TestSuiteConstants.intake = new CanDevices(Constants.IntakeConstants.INTAKE_MOTOR_ID , "Intake", hopper.INTAKE_MOTOR);
        Constants.TestSuiteConstants.rightDriveLeader = new CanDevices(Constants.DriveConstants.RIGHT_DRIVE_FRONT_ID, "Right Drive Leader", drive.rightTalon);
        Constants.TestSuiteConstants.rightDriveFollower = new CanDevices(Constants.DriveConstants.RIGHT_DRIVE_MIDDLE_ID, "Right Drive Follower", drive.rightTalonSlave);
        Constants.TestSuiteConstants.feeder = new CanDevices(Constants.HopperConstants.HOPPER_FEEDER_MOTOR_ID, "Feeder", hopper.HOPPER_FEEDER_MOTOR);
        Constants.TestSuiteConstants.PDP = new CanDevices(0, "PDP", pdp);
        Constants.TestSuiteConstants.leftDriveLeader = new CanDevices(Constants.DriveConstants.LEFT_DRIVE_FRONT_ID, "Left Drive Leader", drive.leftTalon);
        Constants.TestSuiteConstants.leftDriveFollower = new CanDevices(Constants.DriveConstants.LEFT_DRIVE_MIDDLE_ID, "Left Drive Follower", drive.leftTalonSlave);
        Constants.TestSuiteConstants.armLeader = new CanDevices(Constants.ArmConstants.ARM_MOTOR_LEADER_ID, "Arm Leader", arm.ARM_MOTOR_LEADER);
        Constants.TestSuiteConstants.armFollower = new CanDevices(Constants.ArmConstants.ARM_MOTOR_FOLLOWER_ID, "Arm Follower", arm.ARM_MOTOR_FOLLOWER);
        Constants.TestSuiteConstants.shooterLeft = new CanDevices(Constants.ShooterConstants.SHOOTER_MOTOR_LEFT_ID, "Shooter Left", shooter.LEFT_SHOOTER);
        Constants.TestSuiteConstants.gatekeeper = new CanDevices(Constants.HopperConstants.GATEKEEPER_MOTOR_ID, "Gatekeeper", hopper.GATEKEEPER_MOTOR);
        Constants.TestSuiteConstants.shooterRight = new CanDevices(Constants.ShooterConstants.SHOOTER_MOTOR_RIGHT_ID, "Shooter Right", shooter.RIGHT_SHOOTER);
        Constants.TestSuiteConstants.corner = new CanDevices(Constants.HopperConstants.CORNER_MOTOR_ID, "Corner", hopper.CORNER_MOTOR);
        

        CanChain[0] = Constants.TestSuiteConstants.intake;
        CanChain[1] = Constants.TestSuiteConstants.rightDriveLeader;
        CanChain[2] = Constants.TestSuiteConstants.rightDriveFollower;
        CanChain[3] = Constants.TestSuiteConstants.feeder;
        CanChain[4] = Constants.TestSuiteConstants.PDP;
        CanChain[5] = Constants.TestSuiteConstants.leftDriveLeader;
        CanChain[6] = Constants.TestSuiteConstants.leftDriveFollower;
        CanChain[7] = Constants.TestSuiteConstants.armLeader;
        CanChain[8] = Constants.TestSuiteConstants.armFollower;
        CanChain[9] = Constants.TestSuiteConstants.shooterLeft;
        CanChain[10] = Constants.TestSuiteConstants.gatekeeper;
        CanChain[11] = Constants.TestSuiteConstants.shooterRight;
        CanChain[12] = Constants.TestSuiteConstants.corner;
    }

    @Override
    protected void constructHardware() {
        scheduler.schedule(drive, executor);
        scheduler.schedule(hopper, executor);
        scheduler.schedule(shooter, executor);
        scheduler.schedule(arm, executor);
        //scheduler.schedule(robotTracker, executor);

        driveCmdRunning = new DriveCommandRunning();

        ahrs = drive.ahrs;

        // // Instatiator if we're using the NavX
        // gyro = new NavX();

        // // Instatiator if we're using the KoP Gyro
        // gyro = new AnalogDevicesGyro();
        // //gyro.recalibrate();

        joystickRight = new Joystick(1);
        listenerRight = new ListenerManager(joystickRight);
        addListenerManager(listenerRight);

        joystickLeft = new Joystick(0);
        listenerLeft = new ListenerManager(joystickLeft);
        addListenerManager(listenerLeft);

        // initialization of limelights

        shooterLimelight = new Limelight("limelight-shooter", 26.0, 0, 0, 30);
        ballLimelight = new Limelight("limelight-c", Constants.VisionConstants.BOTTOM_LIMELIGHT_ANGLE,
                Constants.VisionConstants.BOTTOM_LIMELIGHT_HEIGHT,
                Constants.VisionConstants.BOTTOM_LIMELIGHT_DISTANCE_FROM_FRONT, 14.5 * Length.in);
        limelights = new Limelight[2];

        // initialization of auto test suite

        limelights[0] = shooterLimelight;
        limelights[1] = ballLimelight;

        pdp = new PowerDistributionPanel(0);
        // setCanChain();
        // errorCatcher = new ErrorCatcherUtility(CanChain, limelights, drive);

        // NarwhalDashboard.addButton("ErrorCatcher", (boolean down) -> {
        //     if (down) {               
        //        errorCatcher.testEverything();
        //     }
        // });
        // NarwhalDashboard.addButton("VelocityTester", (boolean down) -> {
        //     if (down) {               
        //       // errorCatcher.velocityTester();

        //     }
        // });
        NarwhalDashboard.addButton("SetStateLong", (boolean down) -> {
                 if (down) {               
                    stateTracker.setState(RobotState.LONG_RANGE);
                 }
             });
        NarwhalDashboard.addButton("SetStateMid", (boolean down) -> {
            if (down) {               
               stateTracker.setState(RobotState.MID_RANGE);
            }
        });
        NarwhalDashboard.addButton("SetStateShort", (boolean down) -> {
            if (down) {               
               stateTracker.setState(RobotState.SHORT_RANGE);
            }
        });
        drive.resetGyro();
    }

    @Override
    protected void constructAutoPrograms() {
        //NarwhalDashboard.addAuto("Simple Auto", new AutoSimple(drive, shooter, arm, hopper, gyro, shooterLimelight, driveCmdRunning, 10000));
    }

    @Override
    protected void setupListeners() {
        listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
        listenerRight.nameControl(ControllerExtreme3D.TRIGGER, "AlignShoot");
        // listenerRight.nameControl(new Button(3), "ClearTracker");
        listenerRight.nameControl(new Button(2), "zeroCallBount");
        listenerRight.nameControl(new Button(3), "loadingStation");
        listenerRight.nameControl(new Button(4), "RezeroArm");
        listenerRight.nameControl(new Button(5), "ZeroShooter");
        listenerRight.nameControl(new Button(6), "ZeroShooter");
        // shooter calibration:
        //listenerRight.nameControl(new Button(5), "runShooterFF");
        // listenerRight.nameControl(new Button(6), "runArmFF");
        // listenerRight.nameControl(new Button(7), "endVoltage");
        listenerRight.nameControl(new Button(7), "setRangeLong");
        listenerRight.nameControl(new Button(8), "RunBalls");
        listenerRight.nameControl(new Button(9), "setRangeMid");
        listenerRight.nameControl(new Button(10), "EjectBalls");
        listenerRight.nameControl(new Button(11), "setRangeShort");
        listenerRight.nameControl(new Button(12), "queueShooter");
        

        //listenerRight.nameControl(new Button(10), "ShooterFF");
        listenerRight.nameControl(new POV(0), "IntakePOV");

        listenerLeft.nameControl(ControllerExtreme3D.TRIGGER, "Climb");
        //listenerLeft.nameControl(new POV(0), "BalancePOV");
        listenerLeft.nameControl(new Button(3), "EjectClimber");
        listenerLeft.nameControl(new Button(4), "EjectClimber");
        listenerLeft.nameControl(new Button(9), "IncrementBallCount");
        listenerLeft.nameControl(new Button(10), "DecrementBallCount");
        listenerLeft.nameControl(new Button(11), "EmergencyReset");
        listenerLeft.nameControl(new Button(12), "EmergencyReset");


        
        /*listenerRight.addButtonDownListener("ShooterFF", () -> {
            shooterFF = new CmdShooterFF(shooter);
            shooterFF.start();
            Log.info("MainCompbot.java", "Shooter FF Starting");
        });*/

        listenerRight.addMultiListener(() -> {
            if (driveCmdRunning.isRunning) {
                double horiz = -0.5 * listenerRight.getAxis("MoveTurn"); //0.7
                double vert = -1.0 * listenerRight.getAxis("MoveForwards");
                double throttle = -1.0 * listenerRight.getAxis("Throttle");

                drive.arcadeDrive(horiz, vert, throttle, true);
            }
        }, "MoveTurn", "MoveForwards", "Throttle");

        listenerRight.addButtonDownListener("AlignShoot", () -> {
            triggerCommand = new CmdAlignShoot(drive, shooter, arm, hopper, ahrs, shooterLimelight, driveCmdRunning,
                    Constants.VisionConstants.TX_OFFSET, 5);
            triggerCommand.start();
            Log.info("MainCompbot.java", "[Vision Alignment] Started");
        });
        listenerRight.addButtonUpListener("AlignShoot", () -> {
            triggerCommand.cancel();
            triggerCommand = null;
            Log.info("MainCompbot.java", "[Vision Alignment] Stopped");
        });
        listenerRight.addButtonDownListener("loadingStation", () -> {
            Log.info("Button3", "pressed");
            arm.setState(ArmState.LOADING_STATION);

        });
        listenerRight.addButtonDownListener("RezeroArm", () -> {
            Log.info("Button4", "pressed");
            arm.setState(ArmState.STOWED);
        });
        listenerRight.addButtonDownListener("EjectBalls", () -> {
            Log.info("Button10", "pressed");
            //Log.info("MainCompBot", "Eject not implemented yet");
            // ejectBallsCommand = new CmdEjectBalls(hopper);
            // ejectBallsCommand.start();
            hopper.setAction(ActionState.EJECTING);
        });
        listenerRight.addButtonUpListener("EjectBalls", () -> {
            Log.info("Button10", "released");
            hopper.setAction(ActionState.STANDBY);
        });
        listenerRight.addButtonDownListener("RunBalls", () -> {
            Log.info("Button8", "pressed");
            hopper.setAction(ActionState.RUNNING);
        });
        listenerRight.addButtonUpListener("RunBalls", () -> {
            Log.info("Button8", "released");
            hopper.setAction(ActionState.STANDBY);
        });
        listenerRight.addButtonDownListener("zeroCallBount", () -> {
            hopper.setBallCount(0);
        });
        // listenerRight.addButtonDownListener("runShooterFF", () -> {
        //     Log.info("Button5", "pressed");
        //     Log.info("Shooter", "Start Voltage: " +
        //     String.valueOf(RobotController.getBatteryVoltage()));
        //     shooterFFCommand = new CmdShooterFF(shooter);
        //     shooterFFCommand.start(); 
        // });
        // listenerRight.addButtonUpListener("runShooterFF", () -> {
        //     Log.info("Button5", "released");
        //     Log.info("Button5", "stopping cmdShooterFF");
        //     shooterFFCommand.cancel();
        //     shooterFFCommand = null;
        // });
        listenerRight.addButtonDownListener("setRangeLong", () -> {
            stateTracker.setState(RobotState.LONG_RANGE);
        });
        listenerRight.addButtonDownListener("setRangeMid", () -> {
            stateTracker.setState(RobotState.MID_RANGE);
        });
        listenerRight.addButtonDownListener("setRangeShort", () -> {
            stateTracker.setState(RobotState.SHORT_RANGE);
        });
        listenerRight.addButtonDownListener("queueShooter", () -> {
            shooter.queue();
        });
        listenerRight.addButtonDownListener("ZeroShooter", () -> {
            shooter.setSetpoint(0);
        });

         

        listenerRight.addListener("IntakePOV", (POVValue pov) -> {
            switch (pov.getDirectionValue()) {
                case 8:
                case 7:
                case 1:
                // start intake command
                    // povCommand = new CmdBallIntake(gyro, ballLimelight, hopper, arm,
                    // driveCmdRunning,
                    // Constants.VisionConstants.BALL_PID, Constants.VisionConstants.BLIND_BALL_PID,
                    // Constants.GameConstants.BALL_HEIGHT, Constants.VisionConstants.TX_OFFSET);
                    // driveCmdRunning,
                    if (arm.ARM_STATE == Arm.ArmState.INTAKE) {
                        // povCommand.start();
                        hopper.setAction(Hopper.ActionState.INTAKING);
                    } else {
                        arm.setState(Arm.ArmState.INTAKE);
                    }

                    break;
                    
                case 3:
                case 4:
                case 5:
                    // cancels intaking if pushed forward (on accident)
                    hopper.INTAKE_MOTOR.set(Constants.IntakeConstants.INTAKE_MOTOR_REVERSE_VALUE);

                    break;
                    
                case 0:
                    // povCommand.cancel();
                    // povCommand = null;
                    hopper.setAction(Hopper.ActionState.STANDBY);

                    break;
                default:
                    break;
            }
        });

        listenerLeft.addButtonDownListener("Climb", () -> {
            Log.info("Trigger", "pressed");
            climber.setPower(Constants.ClimberConstants.CLIMB_POWER);
        });
        listenerLeft.addButtonUpListener("Climb", () -> {
            Log.info("Trigger", "released");
            climber.setPower(0.0);
        });
        listenerLeft.addButtonDownListener("EjectClimber", () -> {
            Log.info("Button3/4", "pressed");
            arm.setState(ArmState.CLIMBING);
            climber.setIsClimbing(true);
        });
        listenerLeft.addButtonDownListener("IncrementBallCount", () -> {
            Log.info("Button9", "pressed");
            hopper.setBallCount(hopper.ballCount + 1);
        });
        listenerLeft.addButtonDownListener("DecrementBallCount", () -> {
            Log.info("Button10", "pressed");
            hopper.setBallCount(hopper.ballCount - 1);
        });
        listenerLeft.addButtonDownListener("EmergencyReset", () -> {
            Log.info("MainCompBot", "EMERGENCY RESET PRESSED");
            hopper.setBallCount(0);
            arm.setState(ArmState.STOWED);
            hopper.setAction(ActionState.STANDBY);
            shooter.setSetpoint(0);
            climber.setPower(0);
            climber.setIsClimbing(false);
        });
        /*listenerLeft.addListener("BalancePOV", (POVValue pov) -> {
            switch (pov.getDirectionValue()) {
                
                case 1: 
                case 2:
                case 3:
                    climber.balance(-Constants.ClimberConstants.MOVE_POWER);
                    break;
                case 5:
                case 6:
                case 7: 
                    climber.balance(Constants.ClimberConstants.MOVE_POWER);
                    break;
                default:
                    climber.balance(0.0);
                    break;
            }
        });*/
    }

    @Override
    protected void teleopPeriodic() {
        scheduler.resume();
    }

    double maxLeftSpeed = 0;
    double maxRightSpeed = 0;
    double maxSpeed = 0;
    double minLeftSpeed = 0;
    double minRightSpeed = 0;
    double minSpeed = 0;

    double currentLeftSpeed;
    double currentLeftDistance;
    double currentRightSpeed;
    double currentRightDistance;
    double currentSpeed;
    double currentDistance;
    String currentArmLimitSwitch;
    double currentArmAngle;
    double currentArmPos;
    double currentArmVoltage;
    double currentArmPower;
    double currentBatteryVoltage;
    double currentArmCurrent;
    double currentShooterSpeed;
    double currentShooterPower;
    double currentShooterSetpoint;

    @Override
    protected void updateDashboard() {
        SmartDashboard.putString("Arm Angle", String.valueOf(arm.getAngle()));
        NarwhalDashboard.put("time", DriverStation.getInstance().getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("ball_count", hopper.getBallCount());
        NarwhalDashboard.put("shooting_state", StateTracker.robotState.shooterStateName);

        //Log.info("HOPPER", "" + hopper.SENSOR_1_STATE);

        if (arm.getLimitStatus()) {
            arm.ARM_MOTOR_LEADER.setSelectedSensorPosition(0);
            arm.ARM_MOTOR_FOLLOWER.setSelectedSensorPosition(0);
        }

        currentLeftSpeed = drive.getLeftSpeed();
        // currentLeftDistance = drive.getLeftDistance();
        currentRightSpeed = drive.getRightSpeed();
        // currentRightDistance = drive.getRightDistance();

        currentSpeed = drive.getSpeed();
        // currentDistance = drive.getDistance();

        // currentArmLimitSwitch = String.valueOf(arm.getLimitStatus());
        // currentArmAngle = arm.getAngle();
        // currentArmPos = arm.ARM_MOTOR_LEADER.getSelectedSensorPosition(0);

        // currentArmVoltage = arm.ARM_MOTOR_LEADER.getMotorOutputVoltage();
        // currentArmPower = arm.output;
        // currentArmCurrent = arm.ARM_MOTOR_LEADER.getStatorCurrent();

        // currentShooterSpeed = shooter.getRPM();
        // currentShooterPower = shooter.output;
        // currentShooterSetpoint = shooter.setpoint;

        // SmartDashboard.putString("DriveCmdRunning", "" + driveCmdRunning.isRunning);
        SmartDashboard.putString("ActionState", "" + hopper.actionState);

        // SmartDashboard.putString("Gatekeeper Sensor", String.valueOf(hopper.SENSOR_0.get()));
        // SmartDashboard.putString("Hopper Feeder Sensor", String.valueOf(hopper.SENSOR_1.get()));
        // SmartDashboard.putNumber("Corner Encoder", hopper.CORNER_ENCODER.getPosition());

        // SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage());

        // SmartDashboard.putString("Arm Limit Switch", currentArmLimitSwitch);
        // SmartDashboard.putNumber("Arm Angle", currentArmAngle);
        // SmartDashboard.putNumber("Arm Position", currentArmPos);
        // SmartDashboard.putNumber("Arm Voltage", currentArmVoltage);
        // SmartDashboard.putNumber("Arm Current", currentArmCurrent);
        // SmartDashboard.putNumber("Arm Power", currentArmPower);

        // SmartDashboard.putNumber("Shooter Speed", currentShooterSpeed);
        // SmartDashboard.putNumber("Shooter Power", currentShooterPower);

        // SmartDashboard.putNumber("Gyro Angle", drive.getAngle());
        // SmartDashboard.putNumber("Left Distance", currentLeftDistance);
        // SmartDashboard.putNumber("Right Distance", currentRightDistance);

        // SmartDashboard.putNumber("Distance", currentDistance);

        SmartDashboard.putNumber("Left Velocity", currentLeftSpeed);
        SmartDashboard.putNumber("Right Velocity", currentRightSpeed);

        // SmartDashboard.putNumber("Velocity", drive.getSpeed());

        // SmartDashboard.putNumber("RobotTracker - x:", robotTracker.getOdometry().getTranslation().getX());
        // SmartDashboard.putNumber("RobotTracker - y:", robotTracker.getOdometry().getTranslation().getY());
        // SmartDashboard.putNumber("RobotTracker - theta:", robotTracker.getOdometry().getRotation().getDegrees());

        // maxLeftSpeed = Math.max(maxLeftSpeed, currentLeftSpeed);
        // maxRightSpeed = Math.max(maxRightSpeed, currentRightSpeed);
        // maxSpeed = Math.max(maxSpeed, currentSpeed);
        // minLeftSpeed = Math.min(minLeftSpeed, currentLeftSpeed);
        // minRightSpeed = Math.min(minRightSpeed, currentLeftSpeed);
        // minSpeed = Math.min(minSpeed, currentSpeed);

        // SmartDashboard.putNumber("Max Left Speed", maxLeftSpeed);
        // SmartDashboard.putNumber("Min Left Speed", minLeftSpeed);
        // SmartDashboard.putNumber("Max Right Speed", maxRightSpeed);
        // SmartDashboard.putNumber("Min Right Speed", minRightSpeed);

        // SmartDashboard.putNumber("Max Speed", maxSpeed);
        // SmartDashboard.putNumber("Min Speed", minSpeed);
        
        // SmartDashboard.putNumber("Shooter Setpoint", currentShooterSetpoint);
        // SmartDashboard.putNumber("BALL COUNT", hopper.ballCount);

        // trackerCSV += "\n" + String.valueOf(Timer.getFPGATimestamp() - startTime) + ","
        //         + String.valueOf(robotTracker.getOdometry().translationMat.getX()) + ","
        //         + String.valueOf(robotTracker.getOdometry().translationMat.getY()) + ","
        //         + String.valueOf(robotTracker.getOdometry().rotationMat.getDegrees()) + ","
        //         + String.valueOf(robotTracker.trajOdometry.translationMat.getX()) + ","
        //         + String.valueOf(robotTracker.trajOdometry.translationMat.getY());
    }

    @Override
    protected void teleopInit() {
        scheduler.resume();
        shooterLimelight.setLEDMode(LEDMode.OFF);
        arm.ARM_MOTOR_LEADER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
        arm.ARM_MOTOR_FOLLOWER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
        hopper.setAction(ActionState.STANDBY);
        Log.info("MainCompbot", "TeleopInit has started. Setting arm state to ArmState.STARTING");
        driveCmdRunning.isRunning = true;
    }

    @Override
    protected void autonomousInit() {
        scheduler.resume();
        hopper.setAction(ActionState.STANDBY);
        drive.resetGyro();
        Command auto = new AutoSimple(drive, shooter, arm, hopper, ahrs, shooterLimelight, driveCmdRunning, 10000, scheduler);
        auto.start();
    }

    @Override
    protected void disabledInit() {
        shooterLimelight.setLEDMode(LEDMode.OFF);
        arm.ARM_MOTOR_LEADER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE_DEBUG); //TODO: revert to non-debug for comp
        arm.ARM_MOTOR_FOLLOWER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE_DEBUG);
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainCompbot::new);
    }

    @Override
    public void endCompetition() {
        // TODO Auto-generated method stub

    }
}