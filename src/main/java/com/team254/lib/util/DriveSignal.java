package com.team254.lib.util;

import com.team1816.lib.geometry.Rotation2d;

import java.text.DecimalFormat;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.
 */
public class DriveSignal {
//    private final double mLeftMotor;
//    private final double mRightMotor;
//    private final boolean mBrakeMode;

    private double[] mWheelSpeeds;
    private Rotation2d[] mWheelAzimuths; // Radians!

    public DriveSignal() {
        this(new double[]{0, 0, 0, 0}, new Rotation2d[]{Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity()});
    }

    public DriveSignal(double[] wheelSpeeds, Rotation2d[] wheelAzimuths) {
        mWheelSpeeds = wheelSpeeds;
        mWheelAzimuths = wheelAzimuths;
    }

//    public DriveSignal(double left, double right) {
//        this(left, right, false);
//    }
//
//    public DriveSignal(double left, double right, boolean brakeMode) {
//        mLeftMotor = left;
//        mRightMotor = right;
//        mBrakeMode = brakeMode;
//    }
//
//    public static DriveSignal fromControls(double throttle, double turn) {
//        return new DriveSignal(throttle - turn, throttle + turn);
//    }
//
//
//    public static final DriveSignal NEUTRAL = new DriveSignal(0, 0);

    public double[] getWheelSpeeds() {
        return mWheelSpeeds;
    }

    public Rotation2d[] getWheelAzimuths() {
        return mWheelAzimuths;
    }

//    public static final DriveSignal BRAKE = new DriveSignal(0, 0, true);

//    public double getLeft() {
//        return mLeftMotor;
//    }
//
//    public double getRight() {
//        return mRightMotor;
//    }
//
//    public boolean getBrakeMode() {
//        return mBrakeMode;
//    }

    @Override // Cheesy output of wheel speed of each swerve module
    public String toString() {
        String ret_val = "DriveSignal - \n";
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        for (int i = 0; i < mWheelSpeeds.length; i++) {
            ret_val += "\tWheel " + i + ": Speed - " + mWheelSpeeds[i] + ", Azimuth - " + fmt.format(mWheelAzimuths[i].getDegrees()) + " deg\n";
        }

        return ret_val;
    }
}
