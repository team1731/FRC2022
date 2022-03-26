/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.kTICKS;

public class SwerveModule {
	public static final double kMaxAngularSpeed = Math.PI;
	public CANSparkMax m_driveMotor;
	public SparkMaxPIDController m_driveController;
	public RelativeEncoder m_driveEncoder;

	public CANSparkMax m_turningMotor;
	public SparkMaxPIDController m_turnController;
	public RelativeEncoder m_turnEncoder;
	
	private int id;
	private Boolean isInverted = Boolean.FALSE;
	private DebugValues debugValues;

	/**
	 * Constructs a SwerveModule.
	 *
	 * @param driveMotorChannel   ID for the drive motor.
	 * @param turningMotorChannel ID for the turning motor.
	 */
	public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
		id = driveMotorChannel;
		debugValues = new DebugValues(id);

		if (RobotBase.isReal()) {

			m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
			m_driveMotor.restoreFactoryDefaults();
			m_driveController = m_driveMotor.getPIDController();
			m_driveEncoder = m_driveMotor.getEncoder();


			m_driveMotor.setInverted(true);
			m_driveMotor.setSmartCurrentLimit(40, 40);
			m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 5);  // Velocity is in Kstatus1, 
      		m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      		m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);


			// m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
			// m_driveMotor.config_kP(0, 0.1, 30);
			// m_driveMotor.config_kI(0, 0, 30);
			// m_driveMotor.config_kD(0, 0, 30);
			// m_driveMotor.config_kF(0, 1023.0 / 20660.0, 30);
			m_driveController.setP(0.1);
			m_driveController.setI(0);
			m_driveController.setD(0);
			m_driveController.setFF(1023.0 / 20660.0);
			m_driveController.setOutputRange(-1, 1);
			m_driveController.setSmartMotionMaxVelocity(4000, 0); //RPM
			m_driveController.setSmartMotionMinOutputVelocity(0, 0);
			m_driveController.setSmartMotionMaxAccel(3 * (39.37*60*5.5)/Math.PI*3, 0); //RPM per second first number is meters/sec2
			m_driveController.setSmartMotionAllowedClosedLoopError(50, 0);


			/* Config neutral deadband to be the smallest possible */
			// m_driveMotor.configNeutralDeadband(0.001);

			/* Config the peak and nominal outputs */
			// m_driveMotor.configNominalOutputForward(0, 30);
			// m_driveMotor.configNominalOutputReverse(0, 30);
			// m_driveMotor.configPeakOutputForward(1, 30);
			// m_driveMotor.configPeakOutputReverse(-1, 30);

			// m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5,30);

			m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
			m_turningMotor.restoreFactoryDefaults();
			m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5); //kstatus2 has the position
			m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
			m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
			m_turnController = m_turningMotor.getPIDController();
			m_turnEncoder = m_turningMotor.getEncoder();

			/* Configure Sensor Source for Pirmary PID */
			// m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

			/*
			 * set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %)
			 */
			// m_turningMotor.configNeutralDeadband(0.001, 30);

			/**
			 * Configure Talon FX Output and Sesnor direction accordingly Invert Motor to
			 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
			 * sensor to have positive increment when driving Talon Forward (Green LED)
			 */
			// m_turningMotor.setSensorPhase(false);
			m_turningMotor.setInverted(true);

			m_turnController = m_turningMotor.getPIDController();
			m_turnController.setP(0.2);
			m_turnController.setI(0);
			m_turnController.setD(0);
			m_turnController.setFF(0.2);
			/*
			 * Talon FX does not need sensor phase set for its integrated sensor This is
			 * because it will always be correct if the selected feedback device is
			 * integrated sensor (default value) and the user calls getSelectedSensor* to
			 * get the sensor's position/velocity.
			 * 
			 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
			 * sensor-phase
			 */
			// m_turningMotor.setSensorPhase(true);

			/* Set relevant frame periods to be at least as fast as periodic rate */
			// m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
			// m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, 30);
			// m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 30);

			/* Set the peak and nominal outputs */
			// m_turningMotor.configNominalOutputForward(0, 30);
			// m_turningMotor.configNominalOutputReverse(0, 30);
			// m_turningMotor.configPeakOutputForward(1, 30);
			// m_turningMotor.configPeakOutputReverse(-1, 30);

			/* Set Motion Magic gains in slot0 - see documentation */
			// m_turningMotor.selectProfileSlot(0, 0);
			// m_turningMotor.config_kF(0, 0.2, 30);
			// m_turningMotor.config_kP(0, 0.2, 30);
			// m_turningMotor.config_kI(0, 0, 30);
			// m_turningMotor.config_kD(0, 0, 30);

			/* Set acceleration and vcruise velocity - see documentation */
			// m_turningMotor.configMotionCruiseVelocity(18000, 0);
			// m_turningMotor.configMotionAcceleration(18000, 0);
			// m_turningMotor.configMotionSCurveStrength(2);

			/* Zero the sensor once on robot boot up */
			// m_turningMotor.setSelectedSensorPosition(0, 0, 30);

		} else {
			m_driveMotor = null;
			m_turningMotor = null;
		}

	}

	public SwerveModule() {
		System.err.println("DUMMY SWERVE MODULE HAS BEEN INSTANTIATED");
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// return new SwerveModuleState(m_driveEncoder.getRate(), new
		// Rotation2d(m_turningEncoder.get()));
		double velocity = 0;
		double azimuth = 0;
		if (RobotBase.isReal()) { // RPM/60 is RPS *PI*D is inches/s * 39.37 is meter/s but it's 5.5 ticks/rev
			velocity = (m_driveEncoder.getVelocity() / 204.8 * Math.PI * 3.0) / (39.37 * 4.6666666666);
			azimuth = -m_turnEncoder.getPosition();
		}
		double azimuthPercent = Math.IEEEremainder(azimuth, kTICKS) / kTICKS;

		if (RobotBase.isReal()) {
			// SmartDashboard.putNumber("Module"+id+" Drive Encoder Tick",
			// m_driveEncoder.getPosition());
		}

		return new SwerveModuleState(velocity, new Rotation2d(azimuthPercent * 2.0 * Math.PI));

	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param state Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState state) {

		double azimuth = -state.angle.getDegrees() * kTICKS / 360.0;
		double speedMetersPerSecond = state.speedMetersPerSecond;
		// SmartDashboard.putNumber("SpeedMPS-"+id, speedMetersPerSecond);
		// meters per sec * 39.37 is inches/s * 60 is inches per min / PI*D is RPM * 5.5
		// is ticks
		double drive = (speedMetersPerSecond * 9557.333333 * 39.37) / (3.0 * Math.PI); // ticks per second
		// wheel.set(-angleDegrees/360, speedMetersPerSecond * 16.0 * 39.37 * 60.0 / 3.0
		// / Math.PI);
		double azimuthPosition = 0;
		if (RobotBase.isReal()) {
			// azimuthPosition = m_turningEncoder.getPosition();
			azimuthPosition = m_turnEncoder.getPosition();
		}
		double azimuthError = Math.IEEEremainder(azimuth - azimuthPosition, kTICKS);

		// ********************************************************
		// minimize azimuth rotation, reversing drive if necessary
		// ********************************************************
		// synchronized(isInverted){
		isInverted = Math.abs(azimuthError) > 0.25 * kTICKS;
		if (isInverted) {
			azimuthError -= Math.copySign(0.5 * kTICKS, azimuthError);
			drive = -drive;
		}
		// }

		if (RobotBase.isReal()) {
			double turningMotorOutput = azimuthPosition + azimuthError;
			// m_turningPIDController.setReference(turningMotorOutput,
			// ControlType.kSmartMotion);
			m_turnController.setReference(turningMotorOutput, ControlType.kSmartMotion);
			// m_drivePIDController.setReference(drive, ControlType.kSmartVelocity);
			double targetVelocity_UnitsPer100ms = drive / 10; // ticks per 100 ms
			/* 500 RPM in either direction */
			m_driveController.setReference(targetVelocity_UnitsPer100ms, ControlType.kSmartVelocity);

		//	if (System.currentTimeMillis() % 100 == 0) {
		//		SmartDashboard.putNumber("turningMotorOutput-" + id, turningMotorOutput);
		//		SmartDashboard.putNumber("driveVelocityOutput-" + id, drive);
		//	}

		//	debugValues.update(drive, turningMotorOutput, m_turningMotor.getMotorOutputPercent(),
		//			m_turningMotor.getSelectedSensorVelocity(0), m_driveMotor.getMotorOutputPercent(),
		//			m_driveMotor.getSelectedSensorPosition());
		}

		// SmartDashboard.putNumber("RelativeEncoder"+id,
		// m_turningEncoder.getPosition());
		// SmartDashboard.putNumber("absOffset"+id, offsetFromAbsoluteEncoder);
	}

	/**
	 * Zeros all the SwerveModule encoders.
	 */
	public void resetEncoders(double absoluteEncoderVoltage) {
		// synchronized(isInverted){
		if (RobotBase.isReal()) {
			// m_driveMotor.setSelectedSensorPosition(0, 0, 0);
			m_driveEncoder.setPosition(0);
			m_turnEncoder.setPosition(absoluteEncoderVoltage * 16/3.26);
			// absoluteEncoderVoltage = 0;
			// m_turningMotor.setSelectedSensorPosition(absoluteEncoderVoltage * kTICKS, 0, 0);
		}
	}
	// }

	public DebugValues getDebugValues() {
		return debugValues;
	}

	public class DebugValues {
		public int id;

		public double drive;
		public double turningMotorOutput;
		public double turnAppliedOutput;
		public double turnVelocity;
		public double driveAppliedOutput;
		public double driveVelocity;

		public DebugValues(int id) {
			this.id = id;
		}

		public void update(double drive, double turningMotorOutput, double turnAppliedOutput, double turnVelocity,
				double driveAppliedOutput, double driveVelocity) {
			this.drive = drive;
			this.turningMotorOutput = turningMotorOutput;
			this.turnAppliedOutput = turnAppliedOutput;
			this.turnVelocity = turnVelocity;
			this.driveAppliedOutput = driveAppliedOutput;
			this.driveVelocity = driveVelocity;
		}
	}

}
