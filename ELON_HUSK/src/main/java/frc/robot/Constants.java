/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

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

	public static DoubleSolenoid makeDoubleSolenoidForIds(int pcmChannel, int forward_solenoidId,
			int reverse_solenoidId) {
		// System.out.println("creating solenoid ids " + forward_solenoidId + "-" +
		// reverse_solenoidId + " PCM " + pcmChannel + " CHAN ");
		//return new DoubleSolenoid(pcmChannel, moduleType, forward_solenoidId, reverse_solenoidId);
		return new DoubleSolenoid(pcmChannel, Constants.kPneumaticsType, forward_solenoidId, reverse_solenoidId);
	}

	public static final PneumaticsModuleType kPneumaticsType = PneumaticsModuleType.REVPH;
	
	public static final int kTICKS = 33024; // 16.125 * 2048;

	public static final int kDriverControllerPort = 0;
	public static final int kOperatorControllerPort = 1;

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

	}

	public static final class AutoConstants {
		public static final String kDEFAULT_AUTO_CODE = "F1";
		// DEFAULT AUTO MODE if Drive Team is unable to set the mode via Dashboard
		// NOTE: also useful if trying to run in the simulator!
		// XNDD (X=L,M,R,F) (N=1,2,3,4) (DD=0-99 [optional])
		// XN = one of Mark and Chuck's 10 auto modes plus new "forward" mode F
		// (and if it turns out we need a backward mode, B, we will add it)
		// DD = up to 2 digits (0-9) signifying 2 possible delays (in seconds)
		// 1st D = 0-9 second delay at very beginning of auto
		// 2nd D = 0-9 second delay after first shooting event
		// examples:
		// M1 --> run M1 auto with NO DELAYS
		// M25 --> wait 5 seconds, then run M2 auto
		// M203 --> wait 0 seconds, run M2 with 3-sec delay after 1st shooting
		// F12 --> wait 2 seconds, run "forward" auto mode (robot will drive forward a
		// pre-programmed distance)

		public static final double kMaxSpeedMetersPerSecond = 2.6; // 2.6
		public static final double kMaxAccelerationMetersPerSecondSquared = 2; // 2

		public static final double kPXController = 10.0;
		public static final double kPYController = 10.0;
		public static final double kPThetaController = 3;

	}

	public static final class OpConstants {
		
		//Can IDs for all non-driving motors/motor controllers
		//Shooter/Launcher CAN IDS
		public static final int kMotorCANLaunch = 5;
		public static final int kMotorCANRange = 6;

		//Climber CAN IDs
		public static final int kMotorCANClimber1 = 5;
		public static final int kMotorCANClimber2 = 8;

		//Sequencer CAN IDs
		public static final int kMotorCanSequencer1 = 9;
		public static final int kMotorCanSequencer2 = 10;

		//Intake CAN IDs
		public static final int kMotorCANIntakeR = 6;
		public static final int kMotorCANIntakeL = 7;

		//CAN IDs for non-motor components (PDP/Pneumatics Controller)
		//Power Distribution Pannel CAN IDs
		public static final int kPDPCanID = 20;

		//Pneumatics Panel CAN IDs
		public static final int kPneumaticsCanID = 21;
		
		//FIXME: We need to assign IDs to all of these
		//Solenoids
		public static final int kLeftExtenderID = 0;
		public static final int kRightExtenderID = 1;
		public static final int kGrabberNorth1ID = 2;
		public static final int kGrabberNorth2ID = 3;
		public static final int kGrabberSouth1ID = 4;
		public static final int kGrabberSouth2ID = 5;
		public static final int kLeftSwingerMotorID = 6;
		public static final int kRightSwingerMotorID = 7;
		public static final int kNorthSensorID = 0;
		public static final int kSouthSensorID = 1;

		//AnalogInputs
		public static final double kMinIRVoltage = 1.0;
		public static final double kMaxIRVoltage = 1.5;

		public static final double kMotorIntakeFwdSpeed = 0.8; // forward or backward

		/////// TalonFX parameters
		/**
		 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
		 * we just want the primary one.
		 */
		public static final int kPIDLoopIdx = 0;
		/**
		 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
		 * report to DS if action fails.
		 */
		public static final int kTimeoutMs = 30;
		/**
		 * PID Gains may have to be adjusted based on the responsiveness of control
		 * loop. kF: 1023 represents output value to Talon at 100%, 7200 represents
		 * Velocity units at 100% output
		 * 
		 * kP ORIG=4.0 kI kD kF Iz PeakOut
		 */
		public final static Gains kGains_Velocity = new Gains(0.05, 0, 0, .06, 300, 1.00);
		public final static Gains kGains_Range = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);
		
		public final static int SLOT_0 = 0;
		public final static int SLOT_1 = 1;

		public final static int MMCruiseVelocity = 15000;
		public final static int MMAcceleration = 6000;
		public final static int MMScurve = 4;
		public final static int MaxRange = 10000;
		public final static int MinRange = 100;

		///// End TalonFX
	}

	public static final class VisionConstants {
		// Ensure measurements are in METERS
		public static final double kBoilerTargetTopHeight = 0;
		public static final double kCameraDeadband = 0.0;
		public static final double kEpsilon = 1E-9;
		public static final double kMaxTrackerDistance = 18.0;
		public static final double kMaxGoalTrackAge = 1.0; // cp had 1.0
		public static final double kCameraFrameRate = 30.0;
		public static final int kCameraBaudRate = 115200;

		// Ensure measurements are in METERS
		public static final double kCameraXOffset = 0;
		public static final double kCameraYOffset = 0;
		public static final double kCameraZOffset = 0;
		public static final double kCameraPitchAngleDegrees = 0;
		public static final double kCameraYawAngleDegrees = 0;

		// #region DrivePID
		public static final double kDriveP = 0.05;
		public static final double kDriveI = 0;
		public static final double kDriveD = 0;
		public static final double kDriveMaxSpeed = 0.1;
		public static final double kDriveMaxAcceleration = 0.1;
		public static final double kDriveTolerance = 0.5;
		public static final double kDriveAccelerationTolerance = 0.1;
		// #endregion

		// #region TurnPID
		public static final double kTurnP = 0.12;
		public static final double kTurnI = 0;
		public static final double kTurnD = 0.01;
		public static final double kMaxTurnVelocity = 360;
		public static final double kMaxTurnAcceleration = 360;
		public static final double kTurnToleranceDeg = 5;
		public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
		// #endregion

	}

	public static final class DriveConstantsOrig {
		public static final double kTurnP = 0.5;
		public static final double kTurnI = 0;
		public static final double kTurnD = 0;
		public static final double kTurnToleranceDeg = 5;
		public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
		public static final double kMaxTurnRateDegPerS = 10;
		public static final double kMaxTurnAccelerationDegPerSSquared = 30;

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

	public static final class ButtonConstants {
		
		/**
		 * Climb Buttons - Front Left front toggle(R/L): 6, 7
		 */
		public static final int kClimbUp = 6;
		public static final int kClimbDown = 7;
		
		/**
		 * Robot Mode Buttons - Front Right front toggle(R/L): 8, 9
		 */
		public static final int kRobotModeShoot = 8;
		public static final int kRobotModeClimb = 9;

		/**
		 * Intake Mode Buttons - Front Left Bottom(T/B): 14, 15
		 */
		public static final int kIntakeModeEject = 15;
		public static final int kIntakeModePickup = 16;

		/**
		 * Intake Control Buttons - Top Back Toggles(U/D): 1, 12
			Left Button controls left intake, Right Button controls right intake
			When activated, it extends the intake and starts spinning to intake
			and when retracted, it retracts and stops spinning.
		 */
		public static final int kIntakeLeft = 1;
		public static final int kIntakeRight= 12;
		

		public static final int kVision = XboxConstants.kRBumper;
		public static final int kResetGyro = XboxConstants.kAppMenu;
		public static final int kResetEncoders = XboxConstants.kMenu;
	}

	public static final class JoyStickConstants {
		/**
		 * Pickup Mode JoyStick - Left joystick(U/D): 1
		 */
		public static final int kJoyStickPickupMode = 1;

		/**
		 * Shooter Speed JoyStick - Right joystick(U/D): 4
		 */
		public static final int kJoyStickShooterSpeed = 4;
	}

	public static final class CanSparkMaxConstants {
  		public static final double kP = 5e-5;
		public static final double kI = 1e-6; 
		public static final double kD = 0; 
		public static final double kIz = 0; 
		public static final double kFF = 0.000156; 
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final double maxRPM = 5700;

		public static final double maxVel = 2000; //rpm
		public static final double minVel = 0;
		public static final double maxAcc = 1500;
		public static final double allowedErr = 0;

		public static final int smartMotionSlot = 0;
	}
}
