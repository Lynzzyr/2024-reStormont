// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Constants {
    public static final class kControllers {
		public static final int PRIMARY_CONTROLLER = 0;
		public static final int SECONDARY_CONTROLLER = 1;
		public static final int CALIBRATION_CONTROLLER = 2;
		public static final int TEST_CONTROLLER = 2;
	}

    public static final class kRobot {
		public static final boolean IS_HOME_FIELD = false;
	}

    public static final class kDrive {

		public static final double CURRENT_LIMIT = 150.0;

		public static final ShuffleboardTab DRIVE_SHUFFLEBOARD_TAB = Shuffleboard.getTab("Drive");

		public static final double DRIVE_GEAR_RATIO = 6.75;
		public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

		public static final double MAX_DRIVE_VELOCIY = 4.56; // metres per second
		public static final double MAX_DRIVE_ACCELERATION = 2;
		public static final double MAX_TURN_ANGULAR_VELOCITY = 10 / 2;
		public static final double MAX_TURN_ANGULAR_ACCELERATION = 2 * Math.toRadians(360);

		public static final double TRANSLATION_DEADBAND = 0.125;
		public static final double TARGET_HEADING_DEADBAND = 0.3;
		public static final double MANUAL_ROTATION_DEADBAND = 0.2;

		public static final double DRIVE_RAMP_RATE = 0.6;

		public static final double HEADING_SNAP = Math.toRadians(45);

		public static final double HEADING_P = 3;
		public static final double HEADING_I = 0;
		public static final double HEADING_D = 0;
		public static final double HEADING_FF = 0.4;

		public static final class kAutoPathPlanner {
			public static final double TRANSLATION_P = 2.5;
			public static final double TRANSLATION_I = 0;
			public static final double TRANSLATION_D = 0;

			public static final double ROTATION_P = 5;
			public static final double ROTATION_I = 0;
			public static final double ROTATION_D = 0;
		}

        public static final class kAutoAlign {
			public static final class kPIDDrive {
				public static final double T_CONTROLLER_P = 4.6;
				public static final double T_CONTROLLER_I = 0;
				public static final double T_CONTROLLER_D = .5;
				public static final double T_CONTROLLER_FF = .55;
				public static final double T_CONTROLLER_TOLERANCE = 0.01;
			}

			public static final class kPIDDriveSlow {
				public static final double T_CONTROLLER_P = 2.5;
				public static final double T_CONTROLLER_I = 0;
				public static final double T_CONTROLLER_D = 0.02;
				public static final double T_CONTROLLER_FF = .55;
				public static final double T_CONTROLLER_TOLERANCE = 0.005;
			}

			public static final double R_CONTROLLER_P = 8;
			public static final double R_CONTROLLER_I = 0;
			public static final double R_CONTROLLER_D = 1.5;
			public static final double R_CONTROLLER_FF = .45;
			public static final double ROTATION_TOLERANCE = 0.015;
			public static final double ROTATION_TOLERANCE_CUTOFF = 0.018;

			public static final double REACHED_POSITION_TOLERANCE_ClOSE = 0.1;
			public static final double REACHED_POSITION_TOLERANCE = 0.2;
			public static final double REACHED_POSITION_TIMEOUT_FAST = 300; // ms
			public static final double REACHED_POSITION_TIMEOUT_SLOW = 500;

			public static final boolean AUTO_ALIGN_DEBUG = false;

			public static final class kAprilTags {
				public static final Map<Integer, Double> TRAP_TAG_ROTATIONS = Map.of(11, 2.086, 12, 2.086, 13, 2.086,
						14, 2.086 + 1, 15, 3.0, 16, 2.086);
			}

			public static final double TRAP_POSITION_14 = (2.086 - 2.094395102393195);
			public static final double TRAP_POSITION_15 = 2.086;
			public static final double TRAP_POSITION_16 = (2.086 + 2.094395102393195);

			public static final double TRAP_POSITION_11 = (2.086 - 3.141592653589793);
			public static final double TRAP_POSITION_12 = (2.086 + 2.094395102393195 - 3.141592653589793);
			public static final double TRAP_POSITION_13 = (2.086 - 2.094395102393195 - 3.141592653589793);
		}
	}

    public static final class kWaypoints {
		public static final Pose2d AMP_ZONE_TEST = new Pose2d(14.5, 5.37, new Rotation2d(0, -.5));
		public static final Pose2d AMP_ZONE_BLUE = new Pose2d(1.90, 7.78, new Rotation2d(0, Math.toRadians(-90)));
		public static final Pose2d AMP_ZONE_RED = new Pose2d(14.7, 7.78, new Rotation2d(0, Math.toRadians(-90)));
		public static final Pose2d TRAP_ZONE_15 = new Pose2d(4.26, 4.95, new Rotation2d(0, Math.toRadians(270)));
		public static final double TRAP_OFFSET = 0.34;
		public static final double TRAP_DISTANT_OFFSET = .67;
	}
}