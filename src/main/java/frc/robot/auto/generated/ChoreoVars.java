package frc.robot.auto.generated;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final Distance balls_bottom = Units.Meters.of(1.725);
    public static final Distance balls_width = Units.Meters.of(4.62);
    public static final Distance climber_halfwidth = Units.Meters.of(0.432);
    public static final Distance climber_length = Units.Meters.of(1.105);
    public static final Distance climber_x = Units.Meters.of(1.645);
    public static final Distance climber_y = Units.Meters.of(3.745);
    public static final Distance depot_inset = Units.Meters.of(1.571);
    public static final Distance depot_width = Units.Meters.of(1.067);
    public static final Distance endpos_inset = Units.Meters.of(0.667);
    public static final Distance field_length = Units.Meters.of(16.541);
    public static final Distance field_width = Units.Meters.of(8.069);
    public static final Distance hub_inset = Units.Meters.of(4.626);
    public static final LinearVelocity intake = Units.MetersPerSecond.of(1.5);
    public static final Distance koc_depot_rad = Units.Meters.of(0.533);
    public static final Distance outpost = Units.Meters.of(0.666);
    public static final Distance robot_padding = Units.Meters.of(0.514);
    public static final Distance robot_size = Units.Meters.of(0.927);
    public static final Distance startline = Units.Meters.of(4.029);
    public static final Distance startline_bot = Units.Meters.of(3.489);

    public static final class Poses {
        public static final Pose2d depot_end = new Pose2d(0.514, 5.432, Rotation2d.kZero);
        public static final Pose2d depot_score = new Pose2d(1.75, 5, Rotation2d.kZero);
        public static final Pose2d depot_start = new Pose2d(0.514, 7.013, Rotation2d.kZero);
        public static final Pose2d koc_depot = new Pose2d(0.508, 5.965, Rotation2d.kZero);
        public static final Pose2d outpost_score = new Pose2d(2, 2.5, Rotation2d.kZero);
        public static final Pose2d path1_end = new Pose2d(7.762, 6.879, Rotation2d.fromRadians(-1.571));
        public static final Pose2d pos1_score = new Pose2d(3, 5.25, Rotation2d.kZero);
        public static final Pose2d pos3_score = new Pose2d(3, 2.75, Rotation2d.kZero);

        private Poses() {}
    }

    private ChoreoVars() {}
}