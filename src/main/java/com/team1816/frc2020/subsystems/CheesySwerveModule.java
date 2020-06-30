package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.team1816.frc2020.Constants;
import com.team1816.lib.geometry.Rotation2d;
import com.team1816.lib.hardware.MotorUtil;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.Subsystem;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CheesySwerveModule extends Subsystem {

    private static final String NAME = "cheesySwerveModule";

    private final SwerveModuleConstants mConstants;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mControlState = ControlState.OPEN_LOOP;
    private IMotorControllerEnhanced driveMotor;
    private IMotorControllerEnhanced azimuthMotor;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private static final int kPIDLoopId = 0;


    public CheesySwerveModule(SwerveModuleConstants constants) {
        super(NAME);

        mConstants = constants;

        driveMotor = factory.getMotor(mConstants.kSubsystemName, mConstants.kDriveMotorName);
        azimuthMotor = factory.getMotor(mConstants.kSubsystemName, mConstants.kAzimuthMotorName);

        // config sensors
        MotorUtil.checkError(
            driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive encoder");
        MotorUtil.checkError(
            azimuthMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth encoder");

        // config azimuth motion
        MotorUtil.checkError(azimuthMotor.config_kP(0, mConstants.kAzimuthKp, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth kp");
        MotorUtil.checkError(azimuthMotor.config_kI(0, mConstants.kAzimuthKi, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth ki");
        MotorUtil.checkError(
            azimuthMotor.config_IntegralZone(0, mConstants.kAzimuthIZone, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth i zone");
        MotorUtil.checkError(azimuthMotor.config_kD(0, mConstants.kAzimuthKd, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth kd");
        MotorUtil.checkError(azimuthMotor.config_kF(0, mConstants.kAzimuthKf, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth kf");
        MotorUtil.checkError(
            azimuthMotor.configMotionCruiseVelocity(mConstants.kAzimuthCruiseVelocity,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth cruise vel");
        MotorUtil.checkError(
            azimuthMotor.configMotionAcceleration(mConstants.kAzimuthAcceleration, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth max acc");
        MotorUtil.checkError(
            azimuthMotor.configAllowableClosedloopError(0, mConstants.kAzimuthClosedLoopAllowableError,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth allowable closed loop error");
        azimuthMotor.selectProfileSlot(0, 0);

        // config azimuth current/voltage settings
        MotorUtil.configCurrentLimit(azimuthMotor, true,
            mConstants.kAzimuthContinuousCurrentLimit,
            mConstants.kAzimuthPeakCurrentLimit,
            mConstants.kAzimuthPeakCurrentDuration);

        MotorUtil.checkError(
            azimuthMotor.configVoltageMeasurementFilter(mConstants.kAzimuthVoltageMeasurementFilter,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth voltage measurement filter");
        MotorUtil.checkError(
            azimuthMotor.configVoltageCompSaturation(mConstants.kAzimuthMaxVoltage, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth voltage comp saturation");
        azimuthMotor.enableVoltageCompensation(true);

        // config azimuth measurement settings
        MotorUtil.checkError(
            azimuthMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                mConstants.kAzimuthStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth status frame 2 period");
        MotorUtil.checkError(
            azimuthMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
                mConstants.kAzimuthStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth status frame 10 period");
        MotorUtil.checkError(
            azimuthMotor.configVelocityMeasurementPeriod(mConstants.kAzimuthVelocityMeasurementPeriod,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth velocity measurement period");
        MotorUtil.checkError(
            azimuthMotor.configVelocityMeasurementWindow(mConstants.kAzimuthVelocityMeasurementWindow,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth velocity measurement window");

        // config drive current/voltage settings
        MotorUtil.configCurrentLimit(driveMotor, true,
            mConstants.kDriveContinuousCurrentLimit,
            mConstants.kDrivePeakCurrentLimit,
            mConstants.kDrivePeakCurrentDuration);

        MotorUtil.checkError(
            driveMotor.configVoltageMeasurementFilter(mConstants.kDriveVoltageMeasurementFilter,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive voltage measurement filter");
        MotorUtil.checkError(
            driveMotor.configVoltageCompSaturation(mConstants.kDriveMaxVoltage, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive voltage comp saturation");
        driveMotor.enableVoltageCompensation(true);

        // config drive measurement settings
        MotorUtil.checkError(
            driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                mConstants.kDriveStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive status frame 2 period");
        MotorUtil.checkError(
            driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
                mConstants.kDriveStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive status frame 10 period");
        MotorUtil.checkError(
            driveMotor.configVelocityMeasurementPeriod(mConstants.kDriveVelocityMeasurementPeriod,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive velocity measurement period");
        MotorUtil.checkError(
            driveMotor.configVelocityMeasurementWindow(mConstants.kDriveVelocityMeasurementWindow,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive velocity measurement window");

        // config general drive settings
        driveMotor.setInverted(mConstants.kInvertDrive);
        driveMotor.setSensorPhase(mConstants.kInvertDriveSensorPhase);
        driveMotor.setNeutralMode(mConstants.kDriveInitNeutralMode);
        MotorUtil.checkError(driveMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to disable drive forward soft limit");
        MotorUtil.checkError(driveMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to disable drive reverse soft limit");

        // config general azimuth settings
        azimuthMotor.setInverted(mConstants.kInvertAzimuth);
        azimuthMotor.setSensorPhase(mConstants.kInvertAzimuthSensorPhase);
        azimuthMotor.setNeutralMode(mConstants.kAzimuthInitNeutralMode);
        MotorUtil.checkError(azimuthMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to disable azimuth forward soft limit");
        MotorUtil.checkError(azimuthMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to disable azimuth reverse soft limit");

        zeroSensors();
    }


    public synchronized void setOpenLoop(double speed, Rotation2d azimuth) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        Rotation2d current = getAngle();

        double raw_error = current.distance(azimuth);
        if (Math.abs(raw_error) > Math.PI) {
            raw_error -= (Math.PI * 2 * Math.signum(raw_error));
        }

        // error is -180 to 180
        // is wheel reversible logic
        if (Math.abs(raw_error) > Math.PI / 2) {
            speed *= -1;
            raw_error -= Math.PI * Math.signum(raw_error);
        }

        double final_setpoint = getRawAngle() + raw_error;
        // double adjusted_speed = speed * Math.abs(Math.cos(raw_error));

        mPeriodicIO.drive_demand = speed;
        mPeriodicIO.azimuth_demand = radiansToEncoderUnits(final_setpoint);
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.drive_encoder_ticks = driveMotor.getSelectedSensorPosition(0);
        mPeriodicIO.distance = (int) encoderUnitsToDistance(mPeriodicIO.drive_encoder_ticks);
        mPeriodicIO.velocity_ticks_per_100ms = driveMotor.getSelectedSensorVelocity(0);
        mPeriodicIO.azimuth_encoder_ticks = azimuthMotor.getSelectedSensorPosition(0)
            - mConstants.kAzimuthEncoderHomeOffset;

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == ControlState.OPEN_LOOP) {
            if (Util.epsilonEquals(mPeriodicIO.drive_demand, 0.0, mConstants.kDriveDeadband)) { // don't move if
                // throttle is 0
                stop();
            } else {
                azimuthMotor.set(ControlMode.MotionMagic,
                    mPeriodicIO.azimuth_demand + mConstants.kAzimuthEncoderHomeOffset);
                driveMotor.set(ControlMode.PercentOutput, mPeriodicIO.drive_demand);
            }
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (CheesySwerveModule.this) {
                    stop();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (CheesySwerveModule.this) {
                    switch (mControlState) {
                        case OPEN_LOOP:
                            break;
                        default:
                            System.out.println("Unexpected control state: " + mControlState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void zeroSensors() {
        driveMotor.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        /* Azimuth Talon should be in absolute mode */
    }

    @Override
    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0.0);
        azimuthMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber(mConstants.kName + " Module: Module Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(mConstants.kName + " Module: Linear Velocity", getLinearVelocity());
        SmartDashboard.putNumber(mConstants.kName + " Module: Distance Driven", mPeriodicIO.distance);

        SmartDashboard.putNumber(mConstants.kName + " Module: Drive Demand", mPeriodicIO.drive_demand);
        SmartDashboard.putNumber(mConstants.kName + " Module: Azimuth Demand",
            Math.toDegrees(Util.bound0To2PIRadians(encoderUnitsToRadians(mPeriodicIO.azimuth_demand))));
        SmartDashboard.putNumber(mConstants.kName + " Module: Azimuth Error",
            Math.toDegrees(encoderUnitsToRadians(mPeriodicIO.azimuth_demand - getAngleEncoderUnits())));

        SmartDashboard.putNumber(mConstants.kName + " Module: Actual Drive Percent Output", getDrivePercentOutput());
        SmartDashboard.putBoolean(mConstants.kName + " Module: Drive Demand Equals Actual",
            Util.epsilonEquals(mPeriodicIO.drive_demand, getDrivePercentOutput()));

        SmartDashboard.putBoolean(mConstants.kName + " Module: Azimuth At Target", isAzimuthAtTarget());
        SmartDashboard.putNumber(mConstants.kName + " Module: Current", MotorUtil.getSupplyCurrent(azimuthMotor));
        SmartDashboard.putNumber(mConstants.kName + " Module: Voltage", azimuthMotor.getMotorOutputVoltage());

        SmartDashboard.putNumber(mConstants.kName + " Module: Azimuth Absolute Encoder Reading",
            azimuthMotor.getSelectedSensorPosition(kPIDLoopId));

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    /**
     * @param ticks azimuth ticks
     */
    public synchronized double encoderUnitsToRadians(double ticks) {
        return ticks / mConstants.kAzimuthTicksPerRadian;
    }

    /**
     * @return azimuth ticks
     */
    public synchronized double radiansToEncoderUnits(double radians) {
        return radians * mConstants.kAzimuthTicksPerRadian;
    }

    /**
     * @param ticks drive ticks
     */
    public synchronized double encoderUnitsToDistance(double ticks) {
        return ticks * mConstants.kDriveTicksPerUnitDistance;
    }

    /**
     * @return drive ticks
     */
    public synchronized double distanceToEncoderUnits(double distance) {
        return distance / mConstants.kDriveTicksPerUnitDistance;
    }

    public synchronized double getAngleEncoderUnits() {
        return mPeriodicIO.azimuth_encoder_ticks;
    }

    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromRadians((encoderUnitsToRadians(getAngleEncoderUnits())));
    }

    public synchronized double getRawAngle() {
        return encoderUnitsToRadians(getAngleEncoderUnits());
    }

    public synchronized double getUnwrappedAngleDegrees() {
        return Math.toDegrees(encoderUnitsToRadians(getAngleEncoderUnits()));
    }

    public synchronized double getRawLinearVelocity() {
        return mPeriodicIO.velocity_ticks_per_100ms * 10;
    }

    public synchronized double getLinearVelocity() {
        return encoderUnitsToDistance(getRawLinearVelocity());
    }

    public synchronized void setDriveBrakeMode(boolean brake_mode) {
        driveMotor.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized void setAzimuthBrakeMode(boolean brake_mode) {
        azimuthMotor.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized double getDrivePercentOutput() {
        return driveMotor.getMotorOutputPercent();
    }

    public synchronized boolean isAzimuthAtTarget() {
        return Util.epsilonEquals(mPeriodicIO.azimuth_demand, getAngleEncoderUnits(),
            mConstants.kAzimuthClosedLoopAllowableError);
    }

    public enum ControlState {
        OPEN_LOOP
    }

    public static class PeriodicIO {
        // INPUTS
        public double drive_encoder_ticks;
        public double azimuth_encoder_ticks; // actual position of module in encoder units, adjusted for home offset
        public int position_ticks;
        public int distance;
        public int velocity_ticks_per_100ms;

        // OUTPUTS
        public double drive_demand;
        public double azimuth_demand; // actual desired demand in encoder units, not adjusted for home offset
    }

    public static class SwerveModuleConstants {
        public String kName = "Name";
        public String kSubsystemName = Drive.getInstance().getName();
        @Deprecated public int kDriveTalonId = -1;
        public String kDriveMotorName = "";
        @Deprecated public int kAzimuthTalonId = -1;
        public String kAzimuthMotorName = "";

        // general azimuth
        public boolean kInvertAzimuth = false;
        public boolean kInvertAzimuthSensorPhase = false;
        public NeutralMode kAzimuthInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        public double kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI); // for azimuth
        public double kAzimuthEncoderHomeOffset = 0;

        // azimuth motion
        public double kAzimuthKp = 1.3;
        public double kAzimuthKi = 0.05;
        public double kAzimuthKd = 20;
        public double kAzimuthKf = 0.5421;
        public int kAzimuthIZone = 25;
        public int kAzimuthCruiseVelocity = 1698;
        public int kAzimuthAcceleration = 20379; // 12 * kAzimuthCruiseVelocity
        public int kAzimuthClosedLoopAllowableError = 5;

        // azimuth current/voltage
        public int kAzimuthContinuousCurrentLimit = 30; // amps
        public int kAzimuthPeakCurrentLimit = 60; // amps
        public int kAzimuthPeakCurrentDuration = 200; // ms
        public boolean kAzimuthEnableCurrentLimit = true;
        public double kAzimuthMaxVoltage = 10.0; // volts
        public int kAzimuthVoltageMeasurementFilter = 8; // # of samples in rolling average

        // azimuth measurement
        public int kAzimuthStatusFrame2UpdateRate = 10; // feedback for selected sensor, ms
        public int kAzimuthStatusFrame10UpdateRate = 10; // motion magic, ms
        public VelocityMeasPeriod kAzimuthVelocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
        public int kAzimuthVelocityMeasurementWindow = 64; // # of samples in rolling average

        // general drive
        public boolean kInvertDrive = true;
        public boolean kInvertDriveSensorPhase = false;
        public NeutralMode kDriveInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        public double kWheelDiameter = 4.0; // Probably should tune for each individual wheel maybe
        public double kDriveTicksPerUnitDistance = (1.0 / 4096.0) * (18.0 / 28.0 * 15.0 / 45.0)
            * (Math.PI * kWheelDiameter);
        public double kDriveDeadband = 0.01;

        // drive current/voltage
        public int kDriveContinuousCurrentLimit = 30; // amps
        public int kDrivePeakCurrentLimit = 50; // amps
        public int kDrivePeakCurrentDuration = 200; // ms
        public boolean kDriveEnableCurrentLimit = true;
        public double kDriveMaxVoltage = 10.0; // volts
        public int kDriveVoltageMeasurementFilter = 8; // # of samples in rolling average

        // drive measurement
        public int kDriveStatusFrame2UpdateRate = 15; // feedback for selected sensor, ms
        public int kDriveStatusFrame10UpdateRate = 200; // motion magic, ms
        public VelocityMeasPeriod kDriveVelocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
        public int kDriveVelocityMeasurementWindow = 64; // # of samples in rolling average
    }

}
