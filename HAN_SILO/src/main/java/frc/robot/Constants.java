/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int kTICKS = 33024; // 16.125 * 2048;

	public static final double kFlywheelVelocityTolerance = 5 / 100; // percent

	public static final class DriveConstants {

		// Drive motor CAN IDs
		public static final int kLeftFrontDriveMotorPort = 1;
		public static final int kRightFrontDriveMotorPort = 2;
		public static final int kLeftRearDriveMotorPort = 3;
		public static final int kRightRearDriveMotorPort = 4;

		// Turn motor CAN IDs
		public static final int kLeftFrontTurningMotorPort = 11;
		public static final int kRightFrontTurningMotorPort = 12;
		public static final int kLeftRearTurningMotorPort = 13;
		public static final int kRightRearTurningMotorPort = 14;

		public static final double kTrackWidth = 0.7112;
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = 0.7;
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( // leftFront,
																								// rightFront, leftRear,
																								// rightRear
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		public static final boolean kGyroReversed = true; // 09FEB false;

		public static final double kMaxSpeedMetersPerSecond = 4.0; // tune
		public static final double kTurnP = 0.1; // was 0.05
		public static final double kTurnI = 0;
		public static final double kTurnD = 0;

		public static final double kMaxTurnVelocity = 360; // KEEP THIS
		public static final double kMaxTurnAcceleration = 360; // KEEP THIS
		public static final double kTurnToleranceDeg = 5; // KEEP THIS
		public static final double kTurnRateToleranceDegPerS = 10; // KEEP THIS // degrees per second

	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static final class XboxConstants {
		public static final int kA = 1;
		public static final int kB = 2;
		public static final int kX = 3;
		public static final int kY = 4;
		public static final int kLBumper = 5;
		public static final int kRBumper = 6;
		public static final int kAppMenu = 7;
		public static final int kMenu = 8;
		public static final int kLStickClick = 9;
		public static final int kRStickClick = 10;

		public static final int kLStickXAxis = 0;
		public static final int kLStickYAxis = 1;
		public static final int kLTrigger = 2;
		public static final int kRTrigger = 3;
		public static final int kRStickXAxis = 4;
		public static final int kRStickYAxis = 5;

	}
}
