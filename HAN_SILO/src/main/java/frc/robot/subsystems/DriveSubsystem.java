/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.ReflectingCSVWriter;
import frc.robot.util.SwerveModuleDebug;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
	private final Timer m_timer = new Timer();

	private final ReflectingCSVWriter<SwerveModuleDebug> m_CSVWriter;

	private double desiredHeading;

	private final ProfiledPIDController headingController = new ProfiledPIDController(DriveConstants.kTurnP,
			DriveConstants.kTurnI, DriveConstants.kTurnD,
			new TrapezoidProfile.Constraints(DriveConstants.kMaxTurnVelocity, DriveConstants.kMaxTurnAcceleration));

	private boolean headingOverride = true;

	private final AnalogInput leftFrontAbsEncoder;
	private final AnalogInput rightFrontAbsEncoder;
	private final AnalogInput leftRearAbsEncoder;
	private final AnalogInput rightRearAbsEncoder;

	private double driveSpeedScaler = 1.0;

	private Double lockedHeading = null;
	private double m_heading;

	// Robot swerve modules
	private final SwerveModule m_leftFront = new SwerveModule(DriveConstants.kLeftFrontDriveMotorPort,
			DriveConstants.kLeftFrontTurningMotorPort);

	private final SwerveModule m_rightFront = new SwerveModule(DriveConstants.kRightFrontDriveMotorPort,
			DriveConstants.kRightFrontTurningMotorPort);

	private final SwerveModule m_leftRear = new SwerveModule(DriveConstants.kLeftRearDriveMotorPort,
			DriveConstants.kLeftRearTurningMotorPort);

	private final SwerveModule m_rightRear = new SwerveModule(DriveConstants.kRightRearDriveMotorPort,
			DriveConstants.kRightRearTurningMotorPort);

	// The gyro sensor
	// private final Gyro a_gyro = new ADXRS450_Gyro();
	private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

	// Odometry class for tracking robot pose
	private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getAngle());

	public void updateOdometry() {
		if (m_odometry != null) {
			m_odometry.update(new Rotation2d(Math.toRadians(getHeading())), m_leftFront.getState(),
					m_rightFront.getState(), m_leftRear.getState(), m_rightRear.getState());
		}
	}

	/**
	 * Creates a new DriveSubsystem.
	 */
	public DriveSubsystem() {

		leftFrontAbsEncoder = new AnalogInput(0);
		rightFrontAbsEncoder = new AnalogInput(1);
		leftRearAbsEncoder = new AnalogInput(2);
		rightRearAbsEncoder = new AnalogInput(3);

		if (RobotBase.isReal()) {
			if (leftFrontAbsEncoder == null || rightFrontAbsEncoder == null || leftRearAbsEncoder == null
					|| rightRearAbsEncoder == null) {
				System.err.println("\n\nAt least one absolute encoder (AnalogInput(0)--AnalogInput(3) is NULL!!!\n\n");
			}
		}
		headingController.setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
		headingController.enableContinuousInput(-180, 180);

		m_CSVWriter = new ReflectingCSVWriter<>(SwerveModuleDebug.class);
		m_timer.reset();
		m_timer.start();
	}

	public void setDriveSpeedScaler(double axis) {
		this.driveSpeedScaler = 0.5 * (axis + 1);
	}

	/**
	 * Returns the angle of the robot as a Rotation2d.
	 *
	 * @return The angle of the robot.
	 */
	public Rotation2d getAngle() {
		// Negating the angle because WPILib gyros are CW positive.
		return Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
	}

	@Override
	public void periodic() {
		resumeCSVWriter();

		// Update the odometry in the periodic block
		// updateOdometry();

		if (Robot.doSD()) {
			SmartDashboard.putNumber("pose x", m_odometry.getPoseMeters().getTranslation().getX());
			SmartDashboard.putNumber("pose y", m_odometry.getPoseMeters().getTranslation().getY());
			SmartDashboard.putNumber("rot deg", m_odometry.getPoseMeters().getRotation().getDegrees());
			SmartDashboard.putNumber("heading radians", Math.toRadians(getHeading()));
			SmartDashboard.putNumber("raw gyro", m_gyro.getAngle());
			SmartDashboard.putBoolean("gyro is calibrating", m_gyro.isCalibrating());
			SmartDashboard.putNumber("Heading", m_heading);
			SmartDashboard.putNumber("LF speed m/s", m_leftFront.getState().speedMetersPerSecond);
			SmartDashboard.putNumber("LF azimuth", m_leftFront.getState().angle.getDegrees());
		}

		m_CSVWriter.add(new SwerveModuleDebug(m_timer.get(), m_leftFront.getDebugValues(),
				m_rightFront.getDebugValues(), m_leftRear.getDebugValues(), m_rightRear.getDebugValues()));
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(pose, getAngle());
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		drive(xSpeed, ySpeed, 0, 0, fieldRelative);
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rightX, double rightY, boolean fieldRelative) {
		double xSpeedAdjusted = xSpeed;
		double ySpeedAdjusted = ySpeed;
		// double rotAdjusted = rightX;
		double rotationalOutput = rightX;

		// DEADBAND
		if (Math.abs(xSpeedAdjusted) < 0.1) {
			xSpeedAdjusted = 0;
		}
		if (Math.abs(ySpeedAdjusted) < 0.1) {
			ySpeedAdjusted = 0;
		}
		xSpeedAdjusted *= this.driveSpeedScaler;
		ySpeedAdjusted *= this.driveSpeedScaler;

		// If the right stick is neutral - this code should lock on the last known
		// heading
		if (Math.abs(rotationalOutput) < 0.11) {
			headingOverride = true;
			if (lockedHeading == null) {
				headingController.reset(getHeading());
				desiredHeading = getHeading();
				lockedHeading = desiredHeading;
			} else {
				desiredHeading = lockedHeading;
			}
		} else {
			headingOverride = false;
			lockedHeading = null;
			rotationalOutput *= Math.PI;
		}

		if (headingOverride) {
			rotationalOutput = headingController.calculate(getHeading(), desiredHeading);
			SmartDashboard.putNumber("desiredHeading", desiredHeading);
			SmartDashboard.putNumber("headingController Output", rotationalOutput);
		}

		ChassisSpeeds chassisSpeeds;
		if(fieldRelative){
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedAdjusted, ySpeedAdjusted, rotationalOutput, getAngle());
 		} else {
			chassisSpeeds = new ChassisSpeeds(xSpeedAdjusted, ySpeedAdjusted, rotationalOutput);
		}
		// chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedAdjusted, ySpeedAdjusted, rotationalOutput, getAngle())
		// 	: new ChassisSpeeds(xSpeedAdjusted, ySpeedAdjusted, rotationalOutput);

		SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
		setModuleStates(swerveModuleStates);

		SmartDashboard.putNumber("SwerveModuleAzimuthState", swerveModuleStates[0].angle.getDegrees());
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
		m_leftFront.setDesiredState(desiredStates[0]); // leftFront, rightFront, leftRear, rightRear
		m_rightFront.setDesiredState(desiredStates[1]);
		m_leftRear.setDesiredState(desiredStates[2]);
		m_rightRear.setDesiredState(desiredStates[3]);
	}

	/**
	 * Resets the drive encoders to currently read a position of 0.
	 */
	public void resetEncoders() {
		m_leftFront.resetEncoders(leftFrontAbsEncoder.getVoltage() / 3.269); // leftFront, rightFront, leftRear,
																				// rightRear
		m_rightFront.resetEncoders(rightFrontAbsEncoder.getVoltage() / 3.275);// nope! took it back out!// had taken out
																				// but it started working again
																				// 7mar2020. // took this one out -- bad
																				// hardware encoder!!!
		// m_rightFront.resetEncoders(0);// had taken out but it started working again
		// 7mar2020. // took this one out -- bad hardware encoder!!!
		m_leftRear.resetEncoders(leftRearAbsEncoder.getVoltage() / 3.265);
		m_rightRear.resetEncoders(rightRearAbsEncoder.getVoltage() / 3.289);
		resetOdometry(new Pose2d());
	}

	/**
	 * Zeroes the heading of the robot.
	 */
	public void zeroHeading() {
		m_gyro.reset(); // RDB2020 - I replace this call with the below 5 lines...
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		if (m_gyro != null) {
			m_heading = Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
			if (System.currentTimeMillis() % 100 == 0) {
				SmartDashboard.putNumber("Heading", m_heading);
			}
		}
		return m_heading;
	}

	public double getXaccel() {
		return m_gyro.getWorldLinearAccelX() / 9.8066;
	}

	public double getYaccel() {
		return m_gyro.getWorldLinearAccelY() / 9.8066;
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}

	public void suspendCSVWriter() {
		if (!m_CSVWriter.isSuspended()) {
			m_CSVWriter.suspend();
		}
	}

	public void resumeCSVWriter() {
		if (m_CSVWriter.isSuspended()) {
			m_CSVWriter.resume();
		}
	}

	public void displayEncoders() {
		SmartDashboard.putNumber("leftFrontAbsEncoder", leftFrontAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
		SmartDashboard.putNumber("rightFrontAbsEncoder", rightFrontAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
		SmartDashboard.putNumber("leftRearAbsEncoder", leftRearAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
		SmartDashboard.putNumber("rightRearAbsEncoder", rightRearAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
	}

	public void resetGyro() {
		if (m_gyro != null) {
			desiredHeading = 0;
			lockedHeading = null;
			m_gyro.reset();
		}
	}

	public double getXVelocity() {
		return 0;
	}

	public double getYVelocity() {
		return 0;
	}

}
