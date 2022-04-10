// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class LaunchSubsystem extends ToggleableSubsystem {

	// #region ToggleableSubsystem
	@Override
	protected boolean getEnabled() {
		return true;
	}
	// #endregion

	private DoubleSolenoid _LaunchSolenoid;
	private final WPI_TalonFX _RangeMotor;
	private final WPI_TalonFX _LaunchMotor;
	private DutyCycle _absoluteRange;
	private boolean manual_launch = false;
	private double lastPosition = -1.0;
	private int _sdCount = 0;

	private DriveSubsystem m_drive;
	private double _tempAutodistance;
	private boolean _useVision = false;  // just for logging
	private double _joystick;
	private boolean _loggedMessage  = false;

	/**
	 * Creates a new LaunchSubsystem.
	 * 
	 * @param m_drive
	 */
	public LaunchSubsystem(DriveSubsystem drive) {
		if (isDisabled()) {
			_LaunchSolenoid = null;
			_RangeMotor = null;
			_LaunchMotor = null;
			return;
		}

		m_drive = drive;
		_LaunchSolenoid = new DoubleSolenoid(OpConstants.kPneumaticsCanID2, Constants.kPneumaticsType,
				OpConstants.kLaunchOn, OpConstants.kLaunchOff);
		_RangeMotor = new WPI_TalonFX(OpConstants.kMotorCANRange, Constants.kCAN_BUS_CANIVORE);
		_LaunchMotor = new WPI_TalonFX(OpConstants.kMotorCANLaunch, Constants.kCAN_BUS_CANIVORE);

		/* Factory Default Hardware to prevent unexpected behaviour */
		_RangeMotor.configFactoryDefault();
		_LaunchMotor.configFactoryDefault();

		/**
		 * Configure the current limits that will be used
		 * Stator Current is the current that passes through the motor stators.
		 * Use stator current limits to limit rotor acceleration/heat production
		 * Supply Current is the current that passes into the controller from the supply
		 * Use supply current limits to prevent breakers from tripping
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html#current-limit
		 * 
		 * enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)
		 */
		_RangeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 15, 1.0));
		_RangeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 9, 14, 1.0));

		/* setup a basic closed loop */
		_RangeMotor.setNeutralMode(NeutralMode.Coast); // Netural Mode override
		_LaunchMotor.setNeutralMode(NeutralMode.Coast); // Netural Mode override

		_RangeMotor.configSelectedFeedbackSensor(
				TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type
				OpConstants.kPIDLoopIdx, // PID Index
				OpConstants.kTimeoutMs); // Config Timeout

		_LaunchMotor.configSelectedFeedbackSensor(
				TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type
				OpConstants.kPIDLoopIdx, // PID Index
				OpConstants.kTimeoutMs); // Config Timeout

		/*
		 * set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %)
		 */
		_RangeMotor.configNeutralDeadband(0.001, OpConstants.kTimeoutMs);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_RangeMotor.setSensorPhase(true);
		_RangeMotor.setInverted(true);
		_LaunchMotor.setSensorPhase(true);
		_LaunchMotor.setInverted(true);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is
		 * integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
		 * sensor-phase
		 */
		// _RangeMotor.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		// _RangeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,
		// 10, OpConstants.kTimeoutMs);
		// _RangeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
		// 10, OpConstants.kTimeoutMs);

		/* Gains for Position Closed Loop servo */
		_RangeMotor.selectProfileSlot(OpConstants.SLOT_0, OpConstants.kPIDLoopIdx);
		_RangeMotor.config_kP(OpConstants.SLOT_0, OpConstants.kGains_Range.kP, OpConstants.kTimeoutMs);
		_RangeMotor.config_kI(OpConstants.SLOT_0, OpConstants.kGains_Range.kI, OpConstants.kTimeoutMs);
		_RangeMotor.config_kD(OpConstants.SLOT_0, OpConstants.kGains_Range.kD, OpConstants.kTimeoutMs);
		_RangeMotor.config_kF(OpConstants.SLOT_0, OpConstants.kGains_Range.kF, OpConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		_LaunchMotor.config_kF(OpConstants.SLOT_0, OpConstants.kGains_Velocity.kF, OpConstants.kTimeoutMs);
		_LaunchMotor.config_kP(OpConstants.SLOT_0, OpConstants.kGains_Velocity.kP, OpConstants.kTimeoutMs);
		_LaunchMotor.config_kI(OpConstants.SLOT_0, OpConstants.kGains_Velocity.kI, OpConstants.kTimeoutMs);
		_LaunchMotor.config_kD(OpConstants.SLOT_0, OpConstants.kGains_Velocity.kD, OpConstants.kTimeoutMs);

		/**
		 * Phase sensor accordingly. Positive Sensor Reading should match Green
		 * (blinking) Leds on Talon
		 */
		/* Config the peak and nominal outputs */
		_LaunchMotor.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		_LaunchMotor.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		_LaunchMotor.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		_LaunchMotor.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		_RangeMotor.configMotionCruiseVelocity(OpConstants.MMCruiseVelocity, OpConstants.kTimeoutMs);
		_RangeMotor.configMotionAcceleration(OpConstants.MMAcceleration, OpConstants.kTimeoutMs);

		_RangeMotor.configMotionSCurveStrength(OpConstants.MMScurve);

		_absoluteRange = new DutyCycle(new DigitalInput(4));

		stopLaunchBall(); // make sure plunger is out when we start
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (isDisabled()) {
			return;
		}

		if (_sdCount++ > 10) {
			// SmartDashboard.putNumber("absPosition", _absoluteRange.getOutput());
			// SmartDashboard.putNumber("launch_relPos", _RangeMotor.getSelectedSensorPosition());
		//	SmartDashboard.putNumber("_Range%Out", _RangeMotor.getMotorOutputPercent());
			// SmartDashboard.putNumber("_RangeStick", position_0to1);
		//	SmartDashboard.putNumber("_Launch%Out", _LaunchMotor.getMotorOutputPercent());
		//	SmartDashboard.putNumber("_LaunchSpd", _LaunchMotor.getSelectedSensorVelocity());
		//	SmartDashboard.putNumber("_RngLastPos", lastPosition);
		//	SmartDashboard.putNumber("_RngGetPos", _RangeMotor.getSelectedSensorPosition());
		//	SmartDashboard.putNumber("_absRngPos", _absoluteRange.getOutput());
		//	SmartDashboard.putBoolean("_RngManMode", manual_launch);
		//	SmartDashboard.putNumber("AUTO-DISTANCE_METERS", _tempAutodistance);
			
			_sdCount = 0;
		}
	}

	private double normalize_input(double input, double min, double max) {
		double result;
		result = (input - min) / (max - min);
		result = Math.max(0, Math.min(result, 1));

		return result;
	}

	private boolean range_update(double _position, double _percent) {
		boolean update = false;
		if (_position > (lastPosition * (1 + _percent)))
			update = true;
		else if (_position < (lastPosition * (1 - _percent)))
			update = true;
		return update;
	}

	public void manualMode(boolean mode) {
		manual_launch = mode;
	}

	public void resetEncoder() {
		/* Zero the sensor once on robot boot up */
		if (isDisabled()) {
			return;
		}
		calibrateBasket();
	}

	public void resetEncoderAbsolute() {
		if (isDisabled()) {
			return;
		}

		absoluteCalibrateBasket();
	}

	public void runLaunch(double joystick_0to1, double joystick1_0to1,boolean useVision) {
		if (isDisabled()) {
			return;
		}
		_useVision = useVision;
		_joystick = joystick_0to1;
		// SmartDashboard.putNumber("_LaunchJoyPos", position_0to1);
		// SmartDashboard.putNumber("_LaunchJoySpd", speed_0to1);
		//SmartDashboard.putNumber("_RangeStick", joystick_0to1);
		// SmartDashboard.putNumber("_LaunchPercentOut",
		// _LaunchMotor.getMotorOutputPercent());
		// SmartDashboard.putNumber("_LaunchSpeed",
		// _LaunchMotor.getSelectedSensorVelocity());

		double position = 0.0;
		double velUnitsPer100ms = 0.0;
		int index = 0;
		double fraction = 0;

		if (useVision && !m_drive.approximationStale()) { // this method also checks to see if we we are not manually shooting
			// speed_0to1 = getVisionSpeed();
			// position_0to1 = getVisionPosition();
			// position = position_0to1 * OpConstants.MaxRange;
			double distance = m_drive.getApproximateHubDistance();
			if ((distance >= 7 ) || (distance < 0)) {
				distance = 3.0;
				System.out.println("got a distance greater than 8");
			}
			index = (int) distance;
			fraction = distance - index;
		} else {
			fraction = normalize_input(joystick_0to1, 0.226, 0.826) * 7.62;
			_tempAutodistance = fraction;
			index = (int) fraction;
			fraction = fraction - index;


		}

		if (manual_launch) {
			position = normalize_input(joystick1_0to1, 0.226, 0.826) * OpConstants.MaxRange;
		} else {
			position = OpConstants.kRangeArray[index][0]
					+ ((OpConstants.kRangeArray[index + 1][0] - OpConstants.kRangeArray[index][0]) * fraction);
					//System.out.println("calculating position");
		}
		if (position < OpConstants.MinRange) {
			position = OpConstants.MinRange;
		}
	//	if (range_update(position, .05 /* percent tolerance */)) {
			_RangeMotor.set(TalonFXControlMode.MotionMagic, position);
	//		lastPosition = position;
	//	}
		//SmartDashboard.putNumber("_RngCurPos", position);

		/**
		 * Convert 2000 RPM to units / 100ms. Speed - max is 6000.0 RPMs
		 * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
		 * velocity setpoint is in units/100ms
		 */
		/*
		 * normalize_input takes 1) joystick axis input 2) min axis value 3) max axis
		 * value
		 */
		if (manual_launch) {
			velUnitsPer100ms = normalize_input(joystick_0to1, 0.226, 0.826) * 6000 * OpConstants.kVelocity;
		} else {
			velUnitsPer100ms = OpConstants.kRangeArray[index][1]
					+ ((OpConstants.kRangeArray[index + 1][1] - OpConstants.kRangeArray[index][1]) * fraction);
					//System.out.println("setting the velocity");
		}
		_LaunchMotor.set(TalonFXControlMode.Velocity, velUnitsPer100ms);

		//SmartDashboard.putNumber("velUnits/100ms", velUnitsPer100ms);
	}

	// private double getVisionPosition() {
	// // returns a number between 0 and 1 based on distance to the target

	// return (1 - (m_drive.getApproximateHubPosition()/7.62)); //making the last
	// number smaller makes shallower shot
	// }

	// private double getVisionSpeed() {
	// // returns a number between .5 and 1 based on 7.62 meters to 0 meters away
	// from the target. This was just based on what I heard someone say that the
	// shooter speed was beetween .5 and full power
	// return 0.5 + (0.5 * (m_drive.getApproximateHubPosition()/7.62));
	// }

	public void stopLaunch() {
		if (isDisabled()) {
			return;
		}
		_RangeMotor.set(TalonFXControlMode.PercentOutput, 0);
		_LaunchMotor.set(TalonFXControlMode.PercentOutput, 0);
	//	SmartDashboard.putNumber("_LaunchPosition", 0);
	//	SmartDashboard.putNumber("_LaunchSpeed", 0);
	}

	public void runLaunchBall() {
		if (isDisabled()) {
			return;
		}
		_LaunchSolenoid.set(DoubleSolenoid.Value.kForward);
		if (!_loggedMessage)
		System.out.println("Launching-  use Vision:" + _useVision);
		System.out.println("have Lock:" + !m_drive.approximationStale());
		System.out.println("approx dist:" + m_drive.getApproximateHubDistance());
		System.out.println("Basket ticks: "+_RangeMotor.getSelectedSensorPosition()); // 3 meters is 15715 to 16215. Average 15965
		System.out.println("joystick:" + normalize_input(_joystick, 0.226, 0.826) * 7.62);
		_loggedMessage = true;
	}

	public void stopLaunchBall() {
		if (isDisabled()) {
			return;
		}
		_LaunchSolenoid.set(DoubleSolenoid.Value.kReverse);
		_loggedMessage = false;
	}

	private void calibrateBasket() {
		_RangeMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, 0);
	}
	
	private void absoluteCalibrateBasket() {
		System.out.println("Absolute calibration");
		double absPosition = _absoluteRange.getOutput();
		absPosition = normalize_input(absPosition, OpConstants.MinAbsEncoder, OpConstants.MaxAbsEncoder);
		_RangeMotor.setSelectedSensorPosition(absPosition * OpConstants.MaxRange, OpConstants.kPIDLoopIdx, 0);
	}

	public void addTicks() {
		if(isDisabled()) return;

		double tickIncrease = OpConstants.MaxRange * 0.1;
		double currentTicks = _RangeMotor.getSelectedSensorPosition();
		_RangeMotor.setSelectedSensorPosition(currentTicks + tickIncrease, OpConstants.kPIDLoopIdx, 0);
		System.out.println("Adding "+tickIncrease+" ticks. Ticks: "+currentTicks + tickIncrease);
	}

	public void subtractTicks() {
		if(isDisabled()) return;

		double tickDecrease = OpConstants.MaxRange * -0.1;
		double currentTicks = _RangeMotor.getSelectedSensorPosition();
		_RangeMotor.setSelectedSensorPosition(currentTicks + tickDecrease, OpConstants.kPIDLoopIdx, 0);
		System.out.println("Subtracting "+(-tickDecrease)+" ticks. Ticks: "+currentTicks + tickDecrease);
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
