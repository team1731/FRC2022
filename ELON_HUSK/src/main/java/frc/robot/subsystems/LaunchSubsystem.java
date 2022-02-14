// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import frc.robot.Constants.OpConstants;

public class LaunchSubsystem extends ToggleableSubsystem {

  	//#region ToggleableSubsystem
	@Override
	protected boolean getEnabled(){
		return true;
	}
	//#endregion

	private DoubleSolenoid _LaunchSolenoid;
	//private final WPI_TalonFX _RangeMotor;
	private final WPI_TalonFX _LaunchMotor;
	private double lastPosition = -1.0;

  	/** Creates a new LaunchSubsystem. */
  	public LaunchSubsystem() {
		if(isDisabled()){
			_LaunchSolenoid = null;
      		//_RangeMotor = null; 
      		_LaunchMotor = null;
      		return;
		}

		_LaunchSolenoid = null;//Constants.makeDoubleSolenoidForIds(0, OpConstants.k0Launching, OpConstants.k0Climbing);
		//_RangeMotor = new WPI_TalonFX(OpConstants.kMotorCANRange);
		_LaunchMotor = new WPI_TalonFX(OpConstants.kMotorCANLaunch);

		/* Factory Default Hardware to prevent unexpected behaviour */
		//_RangeMotor.configFactoryDefault();
		_LaunchMotor.configFactoryDefault();

		/* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
		//_RangeMotor.configNeutralDeadband(0.001, OpConstants.kTimeoutMs);

		/**
		 * Configure the current limits that will be used
		 * Stator Current is the current that passes through the motor stators.
		 *  Use stator current limits to limit rotor acceleration/heat production
		 * Supply Current is the current that passes into the controller from the supply
		 *  Use supply current limits to prevent breakers from tripping
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html#current-limit
		 * 
		 * enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)  */
		// _RangeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 12, 1.0));
		// _RangeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 2, 3, 0.5));

		/* setup a basic closed loop */
		// _RangeMotor.setNeutralMode(NeutralMode.Brake); // Netural Mode override 
		_LaunchMotor.setNeutralMode(NeutralMode.Coast); // Netural Mode override 

		// _RangeMotor.configSelectedFeedbackSensor(
		// 	TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type 
		// 	OpConstants.kPIDLoopIdx,      			// PID Index
		// 	OpConstants.kTimeoutMs);      			// Config Timeout

		_LaunchMotor.configSelectedFeedbackSensor(
			TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type 
			OpConstants.kPIDLoopIdx,      			// PID Index
			OpConstants.kTimeoutMs);      			// Config Timeout

		/*
			* Talon FX does not need sensor phase set for its integrated sensor
			* This is because it will always be correct if the selected feedback device is integrated sensor (default value)
			* and the user calls getSelectedSensor* to get the sensor's position/velocity.
			* 
			* https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
			*/
		// _RangeMotor.setSensorPhase(true);

		// /* Set relevant frame periods to be at least as fast as periodic rate */
		// _RangeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, OpConstants.kTimeoutMs);
		// _RangeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, OpConstants.kTimeoutMs);
		
		// /* Gains for Position Closed Loop servo */
		// _RangeMotor.config_kP(OpConstants.SLOT_0, OpConstants.kGains_Range.kP, OpConstants.kTimeoutMs);
		// _RangeMotor.config_kI(OpConstants.SLOT_0, OpConstants.kGains_Range.kI, OpConstants.kTimeoutMs);
		// _RangeMotor.config_kD(OpConstants.SLOT_0, OpConstants.kGains_Range.kD, OpConstants.kTimeoutMs);
		// _RangeMotor.config_kF(OpConstants.SLOT_0, OpConstants.kGains_Range.kF, OpConstants.kTimeoutMs);
		
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

		// /* Set acceleration and vcruise velocity - see documentation */
		// _RangeMotor.configMotionCruiseVelocity(OpConstants.MMCruiseVelocity, OpConstants.kTimeoutMs);
		// _RangeMotor.configMotionAcceleration(OpConstants.MMAcceleration, OpConstants.kTimeoutMs);

		// /* Zero the sensor once on robot boot up */
		// _RangeMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, OpConstants.kTimeoutMs);
		// _RangeMotor.configMotionSCurveStrength(OpConstants.MMScurve);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if(isDisabled()){ return; }
	}

	private double normalize_input(double input, double min, double max) {
		double result;
		result = (input - min) / (max - min);
		result = Math.max(0, Math.min(result, 1));
		return result;
	}

	private boolean range_update(double _position, double _percent) {
		boolean update = false;
		if (_position > lastPosition * (1+_percent)) update = true;
		else if (_position < lastPosition * (1 - _percent)) update = true;
		return update;
	}

	public void runLaunch(double speed_0to1, double position_0to1) {
		if(isDisabled()){
			return;
		}
		// SmartDashboard.putNumber("_LaunchJoyPos", position_0to1);
		// SmartDashboard.putNumber("_LaunchJoySpd", speed_0to1);
		//SmartDashboard.putNumber("_RangePercentOut", _RangeMotor.getMotorOutputPercent());
		SmartDashboard.putNumber("_LaunchPercentOut", _LaunchMotor.getMotorOutputPercent());

		/// Range, max range guess is 10000 - OpConstantsMaxRange
		/* normalize_input takes 1) joystick axis input 2) min axis value 3) max axis value */
		double position = normalize_input(position_0to1, 0.140, 0.901) * OpConstants.MaxRange;
		if (position < OpConstants.MinRange) { position = 0; }
		if (range_update(position, .05 /* percent tolerance */)) {
			//_RangeMotor.set(TalonFXControlMode.MotionMagic, position);
			lastPosition = position;
		}
		SmartDashboard.putNumber("_RangeLastPos", lastPosition);
		//SmartDashboard.putNumber("_RangePosition", _RangeMotor.getSelectedSensorPosition());

		/// Speed - max is 6000.0 RPMs
		// launchMotorPercent_0_to_1 *= -1;
		/**
		 * Convert 2000 RPM to units / 100ms.
		 * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
		 * velocity setpoint is in units/100ms
		 */
		/* normalize_input takes 1) joystick axis input 2) min axis value 3) max axis value */
		/* multiply by -1.0 for direction */
		double velUnitsPer100ms = -1.0 * normalize_input(speed_0to1, 0.183, 0.795) * 6000.0 * 2048.0 / 600.0;		
		_LaunchMotor.set(TalonFXControlMode.Velocity, velUnitsPer100ms);
		SmartDashboard.putNumber("velUnitsPer100ms", velUnitsPer100ms);

		/**
		 * Convert 500 RPM to units / 100ms. 2048(FX) 4096(SRX) Units/Rev * 500 RPM /
		 * 600 100ms/min in either direction: velocity setpoint is in units/100ms ==>
		 * 11425 is measured velocity at 80% / 0.8 = 9140/0.8 ==> 3347 is 11425 * 600 *
		 * 2048 == max speed in ticks per 100ms ==> launchPercent is 0 to 1, so 100% ==
		 * put in a value of 1.0
		 */
	}
	
	public void stopLaunch() {
		//_RangeMotor.set(TalonFXControlMode.PercentOutput, 0);
		_LaunchMotor.set(TalonFXControlMode.PercentOutput, 0);
		SmartDashboard.putNumber("_LaunchPostion", 0);
		SmartDashboard.putNumber("_LaunchSpeed", 0);
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
