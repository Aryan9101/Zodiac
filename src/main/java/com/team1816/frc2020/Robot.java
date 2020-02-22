package com.team1816.frc2020;

import badlog.lib.BadLog;
import com.team1816.frc2020.controlboard.ActionManager;
import com.team1816.frc2020.controlboard.ControlBoard;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.*;
import com.team1816.lib.auto.AutoModeExecutor;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.DrivetrainLogger;
import com.team1816.lib.subsystems.Infrastructure;
import com.team1816.lib.subsystems.RobotStateEstimator;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.LatchedBoolean;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Optional;

import static com.team1816.frc2020.controlboard.ControlUtils.*;

public class Robot extends TimedRobot {
    private BadLog logger;

    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    // subsystems
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final LedManager ledManager = LedManager.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Turret turret = Turret.getInstance();
    private final Spinner spinner = Spinner.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final Climber climber = Climber.getInstance();

    // button placed on the robot to allow the drive team to zero the robot right
    // before the start of a match
    DigitalInput resetRobotButton = new DigitalInput(Constants.kResetButtonChannel);

    private boolean mHasBeenEnabled = false;

    private LatchedBoolean mWantsAutoExecution = new LatchedBoolean();
    private LatchedBoolean mWantsAutoInterrupt = new LatchedBoolean();

    private AutoModeSelector mAutoModeSelector = AutoModeSelector.getInstance();
    private AutoModeExecutor mAutoModeExecutor;

    private boolean mDriveByCameraInAuto = false;
    private double loopStart;

    private ActionManager actionManager;
    private CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();
    private AsyncTimer blinkTimer;

    private PowerDistributionPanel pdp = new PowerDistributionPanel();


    Robot() {
        CrashTracker.logRobotConstruction();
    }

    private static RobotFactory factory;

    public static RobotFactory getFactory() {
        if (factory == null) {
            var robotName = System.getenv("ROBOT_NAME");
            if (robotName == null) {
                robotName = "default";
                DriverStation.reportWarning("ROBOT_NAME environment variable not defined, falling back to default.config.yml!", false);
            }
            factory = new RobotFactory(robotName);
        }
        return factory;
    }

    private Double getLastLoop() {
        return (Timer.getFPGATimestamp() - loopStart) * 1000;
    }

    @Override
    public void robotInit() {
        try {
            var logFile = new SimpleDateFormat("MMdd_HH-mm").format(new Date());
            logger = BadLog.init("/home/lvuser/" + System.getenv("ROBOT_NAME") + "_" + logFile + ".bag");

            BadLog.createTopic("Timings/Looper", "ms", mEnabledLooper::getLastLoop, "hide", "join:Timings");
            BadLog.createTopic("Timings/RobotLoop", "ms", this::getLastLoop, "hide", "join:Timings");
            BadLog.createTopic("Timings/Timestamp", "s", Timer::getFPGATimestamp, "xaxis", "hide");


            BadLog.createTopic("Shooter/ActVel", "NativeUnits", shooter::getActualVelocity,
                "hide", "join:Shooter/Velocities");
            BadLog.createTopic("Shooter/TargetVel", "NativeUnits", shooter::getTargetVelocity,
                "hide", "join:Shooter/Velocities");
            BadLog.createTopic("Shooter/Error", "NativeUnits", shooter::getError,
                "hide", "join:Shooter/Velocities");

            BadLog.createTopic("PDP/Current", "Amps", pdp::getTotalCurrent);

            if (Constants.kIsBadlogEnabled) {
                DrivetrainLogger.init(mDrive);

                BadLog.createValue("Drivetrain PID", mDrive.pidToString());
                BadLog.createValue("Shooter PID", shooter.pidToString());
                BadLog.createValue("Turret PID", turret.pidToString());

                BadLog.createTopic("Turret/ActPos", "NativeUnits", () -> (double) turret.getTurretPositionTicks(),
                    "hide", "join:Turret/Positions");
                BadLog.createTopic("Turret/TargetPos", "NativeUnits", turret::getTargetPosition,
                    "hide", "join:Turret/Positions");
                BadLog.createTopic("Turret/ErrorPos", "NativeUnits", turret::getPositionError);

                mDrive.setLogger(logger);
            }

            logger.finishInitialization();

            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                mRobotStateEstimator,
                mDrive,
                mSuperstructure,
                mInfrastructure,
                shooter,
                spinner,
                collector,
                hopper,
                turret
                // climber
            );

            mDrive.zeroSensors();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            ledManager.registerEnabledLoops(mEnabledLooper);
            ledManager.registerEnabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
            mDrive.setHeading(Rotation2d.identity());

            TrajectorySet.getInstance();

            mAutoModeSelector.updateModeCreator();

            actionManager = new ActionManager(
                // Driver Gamepad
                createAction(mControlBoard::getCollectorDown, () -> {
                    hopper.setSpindexer(1);
                    collector.setDeployed(true);
                }),
                createAction(mControlBoard::getCollectorUp, () -> {
                    collector.setDeployed(false);
                    hopper.setSpindexer(0);
                }),

                createScalar(mControlBoard::getDriverClimber, climber::setClimberPower),

                createAction(mControlBoard::getTrenchToFeederSpline, () -> {
                    System.out.println("STARTING TRENCH TO FEEDER");
                    SmartDashboard.putString("Teleop Spline", "TRENCH TO FEEDER");
                    var trajectory = new DriveTrajectory(TrajectorySet.getInstance().TRENCH_TO_FEEDER, true);
                    trajectory.start();
                }),
                createAction(mControlBoard::getFeederToTrenchSpline, () -> {
                    System.out.println("STARTING FEEDER TO TRENCH");
                    SmartDashboard.putString("Teleop Spline", "FEEDER TO TRENCH");
                    turret.setTurretAngle(Turret.CARDINAL_SOUTH);
                    var trajectory = new DriveTrajectory(TrajectorySet.getInstance().FEEDER_TO_TRENCH,true);
                    trajectory.start();
                }),
                createHoldAction(mControlBoard::getSlowMode, mDrive::setSlowMode),

                // Operator Gamepad
                createAction(mControlBoard::getSpinnerReset, spinner::initialize),
                createHoldAction(mControlBoard::getSpinnerColor, spinner::goToColor),
                createHoldAction(mControlBoard::getSpinnerThreeTimes,spinner::spinThreeTimes),

                createAction(mControlBoard::getFeederFlapOut, () -> hopper.setFeederFlap(true)),
                createAction(mControlBoard::getFeederFlapIn, () -> hopper.setFeederFlap(false)),

                createScalar(mControlBoard::getClimber, climber::setClimberPower),

                createHoldAction(mControlBoard::getTurretJogLeft, (moving) -> turret.setTurretSpeed(moving ? -0.2 : 0)),
                createHoldAction(mControlBoard::getTurretJogRight, (moving) -> turret.setTurretSpeed(moving ? 0.2 : 0)),
                createHoldAction(mControlBoard::getAutoHome, turret::setAutoHomeEnabled),
                createHoldAction(mControlBoard::getShoot, (shooting) -> {
                    shooter.setVelocity(shooting ? 10_800 : 0);
                    hopper.lockToShooter(shooting);
                    hopper.setIntake(shooting ? 1 : 0);
                    if (shooting) {
                        mDrive.setOpenLoop(DriveSignal.BRAKE);
                    }
                })
            );

            blinkTimer = new AsyncTimer(
                3, // (3 s)
                () -> ledManager.blinkStatus(LedManager.RobotStatus.ERROR),
                () -> ledManager.indicateStatus(LedManager.RobotStatus.OFF)
            );

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();

            ledManager.indicateStatus(LedManager.RobotStatus.DISABLED);

            // shooter
            shooter.setVelocity(0);

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mInfrastructure.setIsManualControl(false);

            mDisabledLooper.start();

            mDrive.setBrakeMode(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();
            ledManager.indicateStatus(LedManager.RobotStatus.AUTONOMOUS);

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
            mDrive.setHeading(Rotation2d.identity());

            mHasBeenEnabled = true;

            mInfrastructure.setIsManualControl(true); // turn on compressor when superstructure is not moving

            mDrive.zeroSensors();

            System.out.println("Auto init - " + mDriveByCameraInAuto);
            if (!mDriveByCameraInAuto) {
                mAutoModeExecutor.start();
            }

            mEnabledLooper.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);

            // shooter
            // shooter.setVelocity(4200);

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mHasBeenEnabled = true;

            mEnabledLooper.start();

            //turret.setTurretAngle(Turret.CARDINAL_NORTH);
            turret.setTurretPosition(-3960);

            mInfrastructure.setIsManualControl(true);
            mControlBoard.reset();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();

            double initTime = System.currentTimeMillis();

            ledManager.setLedColorBlink(255, 255, 0, 1000);
            // Warning - blocks thread - intended behavior?
            while (System.currentTimeMillis() - initTime <= 3000) {
                ledManager.writePeriodicOutputs();
            }

            mEnabledLooper.stop();
            mDisabledLooper.start();

            blinkTimer.reset();

            ledManager.blinkStatus(LedManager.RobotStatus.DISABLED);

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
            } else {
                System.err.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        try {
            if (!resetRobotButton.get() && !mHasBeenEnabled) {
                System.out.println("Zeroing Robot!");
                mDrive.zeroSensors();
            }

            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            mDriveByCameraInAuto = mAutoModeSelector.isDriveByCamera();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        boolean signalToResume = !mControlBoard.getDrivetrainFlipped(); // TODO: select auto interrupt button
        boolean signalToStop = mControlBoard.getDrivetrainFlipped();
        // Resume if switch flipped up
        if (mWantsAutoExecution.update(signalToResume)) {
            mAutoModeExecutor.resume();
        }

        // Interrupt if switch flipped down
        if (mWantsAutoInterrupt.update(signalToStop)) {
            System.out.println("Auto mode interrupted ");
            mAutoModeExecutor.interrupt();
        }

        if (mDriveByCameraInAuto || mAutoModeExecutor.isInterrupted()) {
            manualControl();
        }
    }

    @Override
    public void teleopPeriodic() {
        loopStart = Timer.getFPGATimestamp();

        try {
            manualControl();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

        if (Constants.kIsBadlogEnabled && Constants.kIsLoggingTeleOp) {
            logger.updateTopics();
            logger.log();
        }
    }

    public void manualControl() {
        // boolean arcadeDrive = false;
        actionManager.update();

        double throttle = mControlBoard.getThrottle();
        double turn = mControlBoard.getTurn();

        DriveSignal driveSignal;

        // if (arcadeDrive) {
        //     var filteredThrottle = Math.signum(throttle) * (throttle * throttle);
        //     double left = Util.limit(filteredThrottle + (turn * 0.55), 1);
        //     double right = Util.limit(filteredThrottle - (turn * 0.55), 1);
        //     driveSignal = new DriveSignal(left, right);
        // } else {
        driveSignal = cheesyDriveHelper.cheesyDrive(throttle, turn, throttle == 0);
        // }
        if (mDrive.getDriveControlState() == Drive.DriveControlState.TRAJECTORY_FOLLOWING) {
            if (driveSignal.getLeft() != 0 || driveSignal.getRight() != 0 || mDrive.isDoneWithTrajectory()) {
                mDrive.setOpenLoop(driveSignal);
            }
        } else {
            mDrive.setOpenLoop(driveSignal);
        }
    }

    @Override
    public void testPeriodic() {
    }
}

