/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import frc.robot.Constants.OpConstants;

public class VelocityTalonFX extends ToggleableSubsystem {

	@Override
	protected boolean getEnabled(){
		return true;
	}

	private final WPI_TalonFX mTalonPickup1;
	//private final WPI_TalonFX mTalonPickup2;
	private double targetVelocity_UnitsPer100ms;

	/**
	 * Creates a new LaunchSubsystem.
	 */
	public VelocityTalonFX() {
		if(isDisabled()){
			mTalonPickup1 = null;
			//mTalonPickup2 = null;
			return;
		}

		mTalonPickup1 = new WPI_TalonFX(6); //OpConstants.kMotorCANIntake1);

		mTalonPickup1.configFactoryDefault();

		// Current limiting
		StatorCurrentLimitConfiguration currentLimitCfg = new StatorCurrentLimitConfiguration(true, 20, 25, 1.0);
		mTalonPickup1.configStatorCurrentLimit(currentLimitCfg);

		/* Config neutral deadband to be the smallest possible */
		mTalonPickup1.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
		mTalonPickup1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
				OpConstants.kPIDLoopIdx, OpConstants.kTimeoutMs);

		/**
		 * Phase sensor accordingly. Positive Sensor Reading should match Green
		 * (blinking) Leds on Talon
		 */
		//mTalonPickup.setSensorPhase(true);

		/* Config the peak and nominal outputs */
		mTalonPickup1.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		mTalonPickup1.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		mTalonPickup1.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		mTalonPickup1.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		mTalonPickup1.config_kF(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kF, OpConstants.kTimeoutMs);
		mTalonPickup1.config_kP(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kP, OpConstants.kTimeoutMs);
		mTalonPickup1.config_kI(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kI, OpConstants.kTimeoutMs);
		mTalonPickup1.config_kD(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kD, OpConstants.kTimeoutMs);

		mTalonPickup1.setNeutralMode(NeutralMode.Coast);

		// SupplyCurrentLimitConfiguration supplyConfig =
		// 		// ENABLED LIMIT(AMP) TRIGGER THRESHOLD(AMP) TRIGGER THRESHOLD TIME(s)
		// 		new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5);
		// // mTalonLaunch1.configSupplyCurrentLimit(supplyConfig);
		// mTalonPickup.configSupplyCurrentLimit(supplyConfig);

		if (System.currentTimeMillis() % 100 == 0) {
			SmartDashboard.putNumber("IntakePercent", 0.5);
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if(isDisabled()){
			return;
		}

		if (System.currentTimeMillis() % 100 == 0) {
		}
	}


	public void spinIntake(double launchMotorPercent_0_to_1) {
		if(isDisabled()){
			return;
		}

		double max = 0.71;
		double min = 0.176;
		launchMotorPercent_0_to_1 = (launchMotorPercent_0_to_1 - min) / (max - min);
		launchMotorPercent_0_to_1 = Math.max(0, Math.min(launchMotorPercent_0_to_1, 1));
		launchMotorPercent_0_to_1 *= -1;

		//targetVelocity_UnitsPer100ms = launchMotorPercent_0_to_1;
			/**
			 * Convert 2000 RPM to units / 100ms.
			 * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
		targetVelocity_UnitsPer100ms = launchMotorPercent_0_to_1 * 2000.0 * 2048.0 / 200.0;
			/* 2000 RPM in either direction */
		//mTalonLaunch2.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
		/**
		 * Convert 500 RPM to units / 100ms. 2048(FX) 4096(SRX) Units/Rev * 500 RPM /
		 * 600 100ms/min in either direction: velocity setpoint is in units/100ms ==>
		 * 11425 is measured velocity at 80% / 0.8 = 9140/0.8 ==> 3347 is 11425 * 600 *
		 * 2048 == max speed in ticks per 100ms ==> launchPercent is 0 to 1, so 100% ==
		 * put in a value of 1.0
		 */
		/* 500 RPM in either direction */
		//mTalonLaunch1.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
		mTalonPickup1.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
		//mTalonLaunch2.set(TalonFXControlMode.Velocity, launchMotorPercent_0_to_1);
		//mTalonLaunch2.set(ControlMode.PercentOutput, launchMotorPercent_0_to_1);

		// if(System.currentTimeMillis() % 100 == 0){
		SmartDashboard.putNumber("mTalonPickup1Inp", targetVelocity_UnitsPer100ms);
		//SmartDashboard.putNumber("talon1Velocity", mTalonLaunch1.getSelectedSensorVelocity());
		SmartDashboard.putNumber("mTalonPickup1Vel", mTalonPickup1.getSelectedSensorVelocity());
	}

	public void stopIntake() {
		if(isDisabled()){
			return;
		}

		System.out.println("LaunchSubsystem: Stopped");

		//mTalonLaunch1.set(ControlMode.PercentOutput, 0);
		//mTalonLaunch2.set(ControlMode.PercentOutput, 0);
		//launchMode();
		mTalonPickup1.set(0);
	}
}

