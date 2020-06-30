package com.team1816.frc2020;

import com.team1816.frc2020.subsystems.Drive;
import com.team1816.lib.geometry.Pose2d;
import com.team1816.lib.geometry.Rotation2d;
import com.team1816.lib.geometry.Twist2d;
import com.team254.lib.util.DriveSignal;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).
 */

public class Kinematics {

    private static final double L = Constants.kDriveWheelbase; //TODO: Constant needs to be made from distance from front of wheels to back
    private static final double W = Constants.kDriveWheelTrackWidthInches;
    private static final double R = Math.hypot(L, W);

    private static final double kEpsilon = 1E-9;

    /**
     * Forward kinematics using only encoders, rotation is implicit (less accurate than below, but useful for predicting
     * motion)
     */
    public static Twist2d forwardKinematics(double left_wheel_delta, double right_wheel_delta) {
        double delta_rotation = (right_wheel_delta - left_wheel_delta) / (Constants.kDriveWheelTrackWidthInches * Constants.kTrackScrubFactor);
        return forwardKinematics(left_wheel_delta, right_wheel_delta, delta_rotation);
    }

    public static Twist2d forwardKinematics(double left_wheel_delta, double right_wheel_delta, double delta_rotation_rads) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        return new Twist2d(dx, 0.0, delta_rotation_rads);
    }

    public static Twist2d forwardKinematics(Rotation2d prev_heading, double left_wheel_delta, double right_wheel_delta,
                                            Rotation2d current_heading) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        final double dy = 0.0;
        return new Twist2d(dx, dy, prev_heading.inverse().rotateBy(current_heading).getRadians());
    }

    /**
     * For convenience, integrate forward kinematics with a Twist2d and previous rotation.
     */
    public static Pose2d integrateForwardKinematics(Pose2d current_pose,
                                                    Twist2d forward_kinematics) {
        return current_pose.transformBy(Pose2d.exp(forward_kinematics));
    }

    /**
     * Uses inverse kinematics to convert a Twist2d into left and right wheel velocities
     */

    public static DriveSignal inverseKinematics(double forward, double strafe, double rotation,
                                                boolean field_relative) {
        return inverseKinematics(forward, strafe, rotation, field_relative, true);
    }

    public static DriveSignal inverseKinematics(double forward, double strafe, double rotation, boolean field_relative,
                                                boolean normalize_outputs) {
        if (field_relative) {
            Rotation2d gyroHeading = Drive.getInstance().getHeading();
            double temp = forward * gyroHeading.cos() + strafe * gyroHeading.sin();
            strafe = -forward * gyroHeading.sin() + strafe * gyroHeading.cos();
            forward = temp;
        }

        double A = strafe - rotation * L / R;
        double B = strafe + rotation * L / R;
        double C = forward - rotation * W / R;
        double D = forward + rotation * W / R;

        double[] wheel_speeds = new double[4];
        wheel_speeds[0] = Math.hypot(B, C);
        wheel_speeds[1] = Math.hypot(B, D);
        wheel_speeds[2] = Math.hypot(A, D);
        wheel_speeds[3] = Math.hypot(A, C);

        // normalize wheel speeds if above 1
        if (normalize_outputs) {
            double max_speed = 1;
            for (int i = 0; i < wheel_speeds.length; i++) {
                if (Math.abs(wheel_speeds[i]) > max_speed) {
                    max_speed = Math.abs(wheel_speeds[i]);
                }
            }

            for (var i = 0; i < wheel_speeds.length; i++) {
                wheel_speeds[i] /= max_speed;
            }
        }

        Rotation2d[] wheel_azimuths = new Rotation2d[4];
        wheel_azimuths[0] = Rotation2d.fromRadians(Math.atan2(B, C));
        wheel_azimuths[1] = Rotation2d.fromRadians(Math.atan2(B, D));
        wheel_azimuths[2] = Rotation2d.fromRadians(Math.atan2(A, D));
        wheel_azimuths[3] = Rotation2d.fromRadians(Math.atan2(A, C));

        return new DriveSignal(wheel_speeds, wheel_azimuths);
    }
}
