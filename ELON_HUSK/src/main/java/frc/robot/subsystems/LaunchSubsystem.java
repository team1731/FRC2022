// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import frc.robot.Constants.OpConstants;
import frc.robot.Constants.InputRange;

public class LaunchSubsystem extends ToggleableSubsystem {

  	//#region ToggleableSubsystem
	@Override
	protected boolean getEnabled(){
		return true;
	}
	//#endregion

	private final WPI_TalonFX _RangeMotor;
	private final WPI_TalonFX _LaunchMotor;
	private boolean _running;

  	/** Creates a new LaunchSubsystem. */
  	public LaunchSubsystem() {
		_running = false; 
		if(isDisabled()){
      		_RangeMotor = null; 
      		_LaunchMotor = null;
      		return;
		}

		_RangeMotor = new WPI_TalonFX(OpConstants.kMotorCANRange);
		_LaunchMotor = new WPI_TalonFX(OpConstants.kMotorCANLaunch);

		/* Factory Default Hardware to prevent unexpected behaviour */
		_RangeMotor.configFactoryDefault();
		_LaunchMotor.configFactoryDefault();
			
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
		_RangeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 21, 1.0));
		_RangeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5, 10, 0.5));

		/* setup a basic closed loop */
		_RangeMotor.setNeutralMode(NeutralMode.Brake); // Netural Mode override 
		_LaunchMotor.setNeutralMode(NeutralMode.Coast); // Netural Mode override 

		/* Config neutral deadband to be the smallest possible */
		_LaunchMotor.configNeutralDeadband(0.001);

		_RangeMotor.configSelectedFeedbackSensor(
			TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type 
			OpConstants.kPIDLoopIdx,      			// PID Index
			OpConstants.kTimeoutMs);      			// Config Timeout

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
		
		/* Gains for Position Closed Loop servo */
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

		SmartDashboard.putString("Debug", "init");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if(isDisabled()){ return; }
		SmartDashboard.putBoolean("_running", _running);
	}

	public void runLaunch(InputRange _input) {
		if(isDisabled()){
			return;
		}
		_running = true;
		setLaunch(_input); 
		SmartDashboard.putString("Debug", "runLaunch");
	}	
	
	public void setLaunch(InputRange _input) {
		if(isDisabled() || (!_running)){
			return;
		}

		SmartDashboard.putNumber("_RangePostion", _input.position);
		SmartDashboard.putNumber("_RangeSpeed", _input.speed);
		SmartDashboard.putString("Debug", "setLaunch");

		_RangeMotor.set(TalonFXControlMode.Position, _input.position);
		_LaunchMotor.set(TalonFXControlMode.Velocity, _input.speed);

		/**
		 * Convert 2000 RPM to units / 100ms.
		 * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
		 * velocity setpoint is in units/100ms
		 */
		// double targetVelocity_UnitsPer100ms = _input.speed * 2000.0 * 2048.0 / 600.0;
		// /**
		//  * Convert 500 RPM to units / 100ms. 2048(FX) 4096(SRX) Units/Rev * 500 RPM /
		//  * 600 100ms/min in either direction: velocity setpoint is in units/100ms ==>
		//  * 11425 is measured velocity at 80% / 0.8 = 9140/0.8 ==> 3347 is 11425 * 600 *
		//  * 2048 == max speed in ticks per 100ms ==> launchPercent is 0 to 1, so 100% ==
		//  * put in a value of 1.0
		//  */
	}
	
	public void stopLaunch() {
		_running = false; 
		_RangeMotor.set(TalonFXControlMode.Velocity, 0);
		_LaunchMotor.set(TalonFXControlMode.Velocity, 0);
		SmartDashboard.putNumber("_RangePostion", 0);
		SmartDashboard.putNumber("_LaunchSpeed", 0);
		SmartDashboard.putString("Debug", "stopLaunch");
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
