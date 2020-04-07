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

public class MainDrivetrain extends NarwhalRobot {

    public Command triggerCommand;
    public Command armFFCommand;
    public Command shooterFFCommand;
    public Command ejectBallsCommand;
    private DriveCommandRunning driveCmdRunning;

    // public StateTracker stateTracker = StateTracker.getInstance();
    static FalconDrive drive = FalconDrive.getInstance();
    // static Hopper hopper = Hopper.getInstance();
    static Arm arm = Arm.getInstance();
    // static Shooter shooter = Shooter.getInstance();
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

    public Command povCommand;
    public Command shooterFF;

    @Override
    protected void constructHardware() {
        scheduler.schedule(drive, executor);
        // scheduler.schedule(hopper, executor);
        // scheduler.schedule(shooter, executor);
        scheduler.schedule(arm, executor);
        //scheduler.schedule(robotTracker, executor);

        driveCmdRunning = new DriveCommandRunning();

        ahrs = drive.ahrs;

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

  
        NarwhalDashboard.addButton("SetStateLong", (boolean down) -> {
                 if (down) {               
                    // stateTracker.setState(RobotState.LONG_RANGE);
                 }
             });
        NarwhalDashboard.addButton("SetStateMid", (boolean down) -> {
            if (down) {               
            //    stateTracker.setState(RobotState.MID_RANGE);
            }
        });
        NarwhalDashboard.addButton("SetStateShort", (boolean down) -> {
            if (down) {               
            //    stateTracker.setState(RobotState.SHORT_RANGE);
            }
        });
        drive.resetGyro();
    }

    @Override
    protected void constructAutoPrograms() {
    }

    @Override
    protected void setupListeners() {
        listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
        listenerRight.nameControl(new Button(3), "loadingStation");
        listenerRight.nameControl(new Button(4), "RezeroArm");
        listenerRight.nameControl(new POV(0), "IntakePOV");

        listenerLeft.nameControl(ControllerExtreme3D.TRIGGER, "Climb");
        listenerLeft.nameControl(new Button(3), "EjectClimber");
        listenerLeft.nameControl(new Button(4), "EjectClimber");
        listenerLeft.nameControl(new Button(11), "EmergencyReset");
        listenerLeft.nameControl(new Button(12), "EmergencyReset");

        listenerRight.addMultiListener(() -> {
            if (driveCmdRunning.isRunning) {
                double horiz = -0.5 * listenerRight.getAxis("MoveTurn"); //0.7
                double vert = -1.0 * listenerRight.getAxis("MoveForwards");
                double throttle = -1.0 * listenerRight.getAxis("Throttle");

                drive.arcadeDrive(horiz, vert, throttle, true);
            }
        }, "MoveTurn", "MoveForwards", "Throttle");
        listenerRight.addButtonDownListener("loadingStation", () -> {
            Log.info("Button3", "pressed");
            arm.setState(ArmState.LOADING_STATION);

        });
        listenerRight.addButtonDownListener("RezeroArm", () -> {
            Log.info("Button4", "pressed");
            arm.setState(ArmState.STOWED);
        });
                 
        listenerRight.addListener("IntakePOV", (POVValue pov) -> {
            switch (pov.getDirectionValue()) {
                case 8:
                case 7:
                case 1:
                    arm.setState(Arm.ArmState.INTAKE);
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
        listenerLeft.addButtonDownListener("EmergencyReset", () -> {
            Log.info("MainCompBot", "EMERGENCY RESET PRESSED");
            arm.setState(ArmState.STOWED);
            climber.setPower(0);
            climber.setIsClimbing(false);
        });
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
    double currentRightSpeed;
    double currentSpeed;
    double currentDistance;
    

    @Override
    protected void updateDashboard() {
        // SmartDashboard.putString("hopper update count", String.valueOf(//hopper.hopper_update_count));
        NarwhalDashboard.put("time", DriverStation.getInstance().getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("ball_count", 0);

        //Log.info("HOPPER", "" + hopper.SENSOR_1_STATE);

        if (arm.getLimitStatus()) {
            arm.ARM_MOTOR_LEADER.setSelectedSensorPosition(0);
            arm.ARM_MOTOR_FOLLOWER.setSelectedSensorPosition(0);
        }

        currentLeftSpeed = drive.getLeftSpeed();
        currentRightSpeed = drive.getRightSpeed();

        currentSpeed = drive.getSpeed();
    

        SmartDashboard.putNumber("Left Velocity", currentLeftSpeed);
        SmartDashboard.putNumber("Right Velocity", currentRightSpeed);

    }

    @Override
    protected void teleopInit() {
        scheduler.resume();
        shooterLimelight.setLEDMode(LEDMode.OFF);
        arm.ARM_MOTOR_LEADER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
        arm.ARM_MOTOR_FOLLOWER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
        Log.info("MainCompbot", "TeleopInit has started. Setting arm state to ArmState.STARTING");
        driveCmdRunning.isRunning = true;
    }

    @Override
    protected void autonomousInit() {
        scheduler.resume();
        drive.resetGyro();

    }

    @Override
    protected void disabledInit() {
        shooterLimelight.setLEDMode(LEDMode.OFF);
        arm.ARM_MOTOR_LEADER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
        arm.ARM_MOTOR_FOLLOWER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainCompbot::new);
    }

    @Override
    public void endCompetition() {
        // TODO Auto-generated method stub

    }
}