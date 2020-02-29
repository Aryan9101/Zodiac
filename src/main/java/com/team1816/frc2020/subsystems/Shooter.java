package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1816.frc2020.Constants;
import com.team1816.lib.hardware.TalonSRXChecker;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Shooter extends Subsystem implements PidProvider {
    private static final String NAME = "shooter";
    private static Shooter INSTANCE;

    private LedManager ledManager = LedManager.getInstance();

    private NetworkTable networkTable;
    private double distance;

    public static Shooter getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Shooter();
        }

        return INSTANCE;
    }

    // Components
    private final IMotorControllerEnhanced shooterMain;
    private final IMotorController shooterFollowerA;
    private final IMotorController shooterFollowerB;
    private final IMotorController shooterFollowerC;

    // State
    private double shooterVelocity;
    private boolean outputsChanged;

    // Constants
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    public static final int MAX_VELOCITY = 11_800; // Far
    public static final int NEAR_VELOCITY = 10_800;  // Initiation line
    public static final int MID_VELOCITY = 9_900 ; // Trench this also worked from initiation
    public static final int VELOCITY_THRESHOLD = (int) factory.getConstant(NAME, "velocityThreshold", 3000);

    private SendableChooser<Integer> velocityChooser = new SendableChooser<>();

    private Shooter() {
        super(NAME);

        this.shooterMain = factory.getMotor(NAME, "shooterMaster");
        this.shooterFollowerA = factory.getMotor(NAME, "shooterFollowerA", shooterMain);
        this.shooterFollowerB = factory.getMotor(NAME, "shooterFollowerB", shooterMain);
        this.shooterFollowerC = factory.getMotor(NAME, "shooterFollowerC", shooterMain);
        //   this.hood = factory.getSolenoid(NAME, "hood");

        this.kP = factory.getConstant(NAME, "kP");
        this.kI = factory.getConstant(NAME, "kI");
        this.kD = factory.getConstant(NAME, "kD");
        this.kF = factory.getConstant(NAME, "kF");

        shooterMain.setNeutralMode(NeutralMode.Coast);
        shooterFollowerA.setNeutralMode(NeutralMode.Coast);
        shooterFollowerB.setNeutralMode(NeutralMode.Coast);
        shooterFollowerC.setNeutralMode(NeutralMode.Coast);

        configCurrentLimits(40 /* amps */);

        shooterMain.setInverted(true);
        shooterFollowerA.setInverted(true);
        shooterFollowerB.setInverted(false);
        shooterFollowerC.setInverted(false);

        shooterMain.configClosedloopRamp(0.5, Constants.kCANTimeoutMs);
        shooterMain.setSensorPhase(false);

        networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");

        networkTable.addEntryListener("distance", (table, key, entry, value, flags) -> {
            distance = value.getDouble();
        },EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    private void configCurrentLimits(int currentLimitAmps) {
        // ((TalonSRX) shooterMain).enableCurrentLimit(true);
        // ((TalonSRX) shooterFollowerA).enableCurrentLimit(true);
        // ((TalonSRX) shooterFollowerB).enableCurrentLimit(true);
        // ((TalonSRX) shooterFollowerC).enableCurrentLimit(true);
        // ((TalonSRX) shooterMain).configContinuousCurrentLimit(currentLimitAmps);
        // ((TalonSRX) shooterFollowerA).configContinuousCurrentLimit(currentLimitAmps);
        // ((TalonSRX) shooterFollowerB).configContinuousCurrentLimit(currentLimitAmps);
        // ((TalonSRX) shooterFollowerC).configContinuousCurrentLimit(currentLimitAmps);
        if (shooterMain instanceof TalonFX) {
            ((TalonFX) shooterMain).configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, currentLimitAmps, 0, 0)
            );
        }
        if (shooterFollowerB instanceof TalonFX) {
            ((TalonFX) shooterFollowerB).configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, currentLimitAmps, 0, 0)
            );
        }
    }

    @Override
    public double getKP() {
        return kP;
    }

    @Override
    public double getKI() {
        return kI;
    }

    @Override
    public double getKD() {
        return kD;
    }

    @Override
    public double getKF() {
        return kF;
    }

    public void setVelocity(double velocity) {
        this.shooterVelocity = velocity;
        outputsChanged = true;
    }

    public void startShooterBasedOnDistance() {
        /* if (distance <  ) { TODO: Find correct distances
            setVelocity(NEAR_VELOCITY);
        } else if (< distance < ) {
            setVelocity(MID_VELOCITY);
        } else if (distance > ) {
            setVelocity(MAX_VELOCITY);
        } */
    }

    public void shootFromChooser() {
        setVelocity(velocityChooser.getSelected());
    }

    public void startShooter() {
        setVelocity(MID_VELOCITY);
    }

    public void stopShooter() {
        setVelocity(0);
    }

    public double getActualVelocity() {
        return shooterMain.getSelectedSensorVelocity(0);
    }

    public double getTargetVelocity() {
        return shooterVelocity;
    }

    public double getError() {
        return shooterMain.getClosedLoopError(0);
    }

    public boolean isVelocityNearTarget() {
        return Math.abs(this.getError()) < VELOCITY_THRESHOLD;
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            if (shooterVelocity == 0) {
                this.shooterMain.set(ControlMode.PercentOutput, 0); // Inertia coast to 0
            } else {
                this.shooterMain.set(ControlMode.Velocity, shooterVelocity);
            }
            outputsChanged = false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Shooter/IsAtSpeed", this::isVelocityNearTarget, null);
        builder.addDoubleProperty("Shooter/ShooterVelocity", this::getActualVelocity, this::setVelocity);

        velocityChooser.setDefaultOption("NEAR_VELOCITY", NEAR_VELOCITY);
        velocityChooser.addOption("MID_VELOCITY", MID_VELOCITY);
        velocityChooser.addOption("MAX_VELOCITY", MAX_VELOCITY);

        SmartDashboard.putData(velocityChooser);
    }

    @Override
    public void stop() {

    }

    private TalonSRXChecker.CheckerConfig getTalonCheckerConfig(IMotorControllerEnhanced talon) {
        return TalonSRXChecker.CheckerConfig.getForSubsystemMotor(this, talon);
    }

    @Override
    public boolean checkSystem() {
        boolean checkShooter = TalonSRXChecker.checkMotors(this,
            new ArrayList<>() {
                {
                    add(new TalonSRXChecker.TalonSRXConfig("shooterMain", shooterMain));
                }
            }, getTalonCheckerConfig(shooterMain));

        System.out.println(checkShooter);
        if (checkShooter){
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
        }
        else {
            ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
        }
        return checkShooter;
    }
}
