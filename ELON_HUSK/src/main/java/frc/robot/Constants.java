/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

	public static final String kCAN_BUS_DEFAULT = "rio";
	public static final String kCAN_BUS_CANIVORE = "Driver CAN Bus";

	public static final PneumaticsModuleType kPneumaticsType = PneumaticsModuleType.REVPH;

	public static final int kTICKS = 33024; // 16.125 * 2048;

	public static final int kDriverControllerPort = 0;
	public static final int kOperatorControllerPort = 1;

	public static final class DriveConstants {

		// Drive motor CAN IDs
		public static final int kLeftFrontDriveMotorPort = 21;
		public static final int kRightFrontDriveMotorPort = 22;
		public static final int kLeftRearDriveMotorPort = 23;
		public static final int kRightRearDriveMotorPort = 24;

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

		public static final double kMaxSpeedMetersPerSecond = 3.5; // tune

		public static final double kTurnP = 0.1; // was 0.05
		public static final double kTurnI = 0;
		public static final double kTurnD = 0;

	}

	public static final class AutoConstants {
		public static final String kDEFAULT_AUTO_CODE = "L4";
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
		public static final String kPATH = "paths/output/";
		public static final double kMaxSpeedMetersPerSecond = 2.6; // 2.6
		public static final double kMaxAccelerationMetersPerSecondSquared = 2; // 2

		public static final double kPXController = 10.0;
		public static final double kPYController = 10.0;
		public static final double kPThetaController = 3;

	}

	public static final class OpConstants {

		// Can IDs for all non-driving motors/motor controllers
		// Shooter/Launcher CAN IDS
		public static final int kMotorCANLaunch = 5;
		public static final int kMotorCANRange = 6;

		// Sequencer CAN IDs
		public static final int kMotorCanSequencer1 = 18;
		public static final int kMotorCanSequencer2 = 19;

		// Intake CAN IDs
		public static final int kMotorCANIntakeR = 8;
		public static final int kMotorCANIntakeL = 7;

		// CAN IDs for non-motor components (PDP/Pneumatics Controller)
		// Power Distribution Pannel CAN IDs
		public static final int kPDPCanID = 1;

		// Pneumatics Panel CAN IDs
		public static final int kPneumaticsCanID = 2;

		// ClimbSubsystem
		public static final int kExtenderUpID = 15;
		public static final int kExtenderDownID = 12;
		public static final int kGrabberNorthFrontOpenID = 6;
		public static final int kGrabberNorthFrontCloseID = 7;
		public static final int kGrabberNorthBackOpenID = 0;
		public static final int kGrabberNorthBackCloseID = 1;
		public static final int kGrabberSouthFrontOpenID = 2;
		public static final int kGrabberSouthFrontCloseID = 3;
		public static final int kGrabberSouthBackOpenID = 5;
		public static final int kGrabberSouthBackCloseID = 4;

		public static final int kLeftSwingerMotorID = 9;
		public static final int kRightSwingerMotorID = 10;
		public static final int kNorthSensorID = 1; // NavX Analog Input Sensor
		public static final int kSouthSensorID = 2; // NavX Analog Input Sensor

		// AnalogInputs
		public static final double kMinIRVoltage = 1.0;
		public static final double kMaxIRVoltage = 1.5;

		public static final double kMotorIntakeFwdSpeed = 0.8; // forward or backward
		public static final double kMotorLeftIntakeSpeed = -1; // backward
		public static final double kMotorRightIntakeSpeed = 1; // forward

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
		public final static Gains kGains_Velocity = new Gains(0.4, 0, 0, .056, 300, 1.00);
		public final static Gains kGains_Range = new Gains(0.5, 0.0, 0.0, 0.2, 0, 1.0);

		public final static int SLOT_0 = 0;
		public final static int SLOT_1 = 1;

		public final static int MMCruiseVelocity = 15000;
		public final static int MMAcceleration = 6000;
		public final static int MMScurve = 1;
		public final static int MaxRange = 31718;
		public final static int MinRange = 0;
		public final static double MinAbsEncoder = 0.028;
		public final static double MaxAbsEncoder = 0.975;

		///// End TalonFX

		///// Begin Pneumatics Constants
		public final static int kLTopA = 10;
		public final static int kLBottomB = 13;
		public final static int kRTopA = 11;
		public final static int kRBottomB = 14;
		public final static int kFTop = 12;
		public final static int kFBottomB = 15;
		public final static int kLaunchOn = 9;
		public final static int kLaunchOff = 8;

		// ltop = left intake top pneumatics: 13
		// lbottom = left intake bottom pneumatics: 10
		// rtop = right intake top pneumatics: 14
		// rbottom = right intake bottom pneumatics: 11
		// ftop = top climber pneumatics: 15
		// fbottom = bottom climber pneumatics: 12

		// Range table for shooting values:Index 2 = Velocity, Index 3 = Position

		// Motor velocity in RPM/100ms
		public static final double kVelocity = 2048 / 600;

		public static final double kRangeArray[][] = { // position ticks, velUnitsPer100ms
				{ 30000.0, 5999.0 }, // 0 meters - eject
				{ 15530.0, 7000.0 }, // 1 meter
				{ 17814.0, 8917.0 }, // 2 meters //35000
				{ 15530.0, 10203.0 }, // 3 meters changed 3/4 //32652
				{ 10971.0, 12758.0 }, // 4 meters
				{  6250.0, 14400.0 }, // 5 meters
				{  1529.0, 16042.0 },  // 6 meters
				{  1000.0, 16042.0 },  // 7 untested
				{ 0.0, 6000.0 },
		};
	}

	public static final class VisionConstants {
		// Ensure measurements are in METERS
		public static final double kMaxTrackerDistance = 18.0;
		public static final double kMaxGoalTrackAge = 1.0; // cp had 1.0
		public static final double kGoalHeight = 2.67;

		// Ensure measurements are in METERS
		public static final double kCameraXOffset = 0;
		public static final double kCameraYOffset = 0;
		public static final double kCameraZOffset = 0;
		public static final double kCameraPitchAngleDegrees = 41.67;
		public static final double kCameraYawAngleDegrees = 0;
		public static final double kCameraLensHeightMeters = 0.7142;

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
		public static final double kTurnD = 0.00;
		public static final double kMaxTurnVelocity = 360;
		public static final double kMaxTurnAcceleration = 360;
		public static final double kTurnToleranceDeg = 5;
		public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
		// #endregion

		public static final double kAverageKeepTime = 0.2;
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
		 * Climb sensor override button - Select Spinner Press: 16
		 */
		public static final int kClimbSensorOverride = 16;

		/**
		 * Robot Mode Buttons - Front Right front toggle(R/L): 8, 9
		 */
		public static final int kRobotModeShoot = 8;
		public static final int kRobotModeClimb = 9;
		public static final int kLaunchManualMode = 10;

		/**
		 * Intake Mode Buttons - Front Left Bottom(T/B): 14, 15
		 */
		public static final int kIntakeLeftEject = 14;
		public static final int kIntakeRightEject = 15;

		/**
		 * Intake Control Buttons - Top Back Toggles(U/D): 1, 12
		 * Left Button controls left intake, Right Button controls right intake
		 * When activated, it extends the intake and starts spinning to intake
		 * and when retracted, it retracts and stops spinning.
		 */
		public static final int kIntakeLeft = 1;
		public static final int kIntakeRight = 12;

		public static final int kVision = XboxConstants.kRBumper;
		public static final int kResetGyro = XboxConstants.kAppMenu;
		public static final int kResetEncoders = XboxConstants.kMenu;
		public static final int kLaunchBall = XboxConstants.kLBumper;
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

	public static final class ClimbConstants {
  		public static final double kP = .0001;
		public static final double kI = 0; 
		public static final double kD = 0; 
		public static final double kIz = 0; 
		public static final double kFF = 0.0006; 
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final double maxRPM = 5700;

		public static final double maxVel = 4000; //rpm
		public static final double minVel = 0;
		public static final double maxAcc = 1000;
		public static final double allowedErr = 0;


		public static final double kBckSteps = 10*4;
		public static final double kStartSteps = 5;
		public static final double kSecondBarSteps = 50 *4;
		public static final double kThirdBarSteps = 120*4;

		public static final int smartMotionSlot = 0;
	}
}
