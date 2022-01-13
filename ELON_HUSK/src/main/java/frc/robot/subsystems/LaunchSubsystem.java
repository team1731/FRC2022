/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;
import frc.robot.Constants.OpConstants;

public class LaunchSubsystem extends ToggleableSubsystem {

	@Override
	protected boolean getEnabled(){
		return true;
	}

	private DoubleSolenoid mLaunchClimbSolenoid;
	private DoubleSolenoid mClimberSolenoid;
	private DoubleSolenoid mLaunchHoodSolenoid;
	private DoubleSolenoid mBrakeSolenoid;
	private final TalonFX mTalonLaunch1;
	private final TalonFX mTalonLaunch2;
	private final DigitalInput sHiCylinder;
	private final DigitalInput sLoCylinder;
	private final DigitalInput sClimbExtend;
	private final DigitalInput sClimbRetract;
	private double targetVelocity_UnitsPer100ms;

	/**
	 * Creates a new LaunchSubsystem.
	 */
	public LaunchSubsystem() {
		if(isDisabled()){
			mLaunchClimbSolenoid = null;
			mClimberSolenoid = null;
			mLaunchHoodSolenoid = null;
			mBrakeSolenoid = null;
			mTalonLaunch1 = null;
			mTalonLaunch2 = null;
			sHiCylinder = null;
			sLoCylinder = null;
			sClimbExtend = null;
			sClimbRetract = null;
			return;
		}

		mLaunchClimbSolenoid = Constants.makeDoubleSolenoidForIds(0, OpConstants.k0Launching, OpConstants.k0Climbing);
		mClimberSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1ClimbRetract, OpConstants.k1ClimbExtend);
		mLaunchHoodSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1HoodRetract, OpConstants.k1HoodExtend);
		mBrakeSolenoid = Constants.makeDoubleSolenoidForIds(0, OpConstants.k0BrakeOn, OpConstants.k0BrakeOff);
		mTalonLaunch1 = new TalonFX(OpConstants.kMotorCANLaunch1);
		mTalonLaunch2 = new TalonFX(OpConstants.kMotorCANLaunch2);

		sHiCylinder = new DigitalInput(OpConstants.kHiCylinder);
		sLoCylinder = new DigitalInput(OpConstants.kLoCylinder);
		sClimbExtend = new DigitalInput(OpConstants.kClimbExtend);
		sClimbRetract = new DigitalInput(OpConstants.kClimbRetract);

		mTalonLaunch1.configFactoryDefault();
		mTalonLaunch2.configFactoryDefault();

		mTalonLaunch1.setInverted(TalonFXInvertType.CounterClockwise);
		mTalonLaunch2.setInverted(TalonFXInvertType.Clockwise);

		/* Config sensor used for Primary PID [Velocity] */
		mTalonLaunch1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, OpConstants.kPIDLoopIdx,
				OpConstants.kTimeoutMs);
		mTalonLaunch2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, OpConstants.kPIDLoopIdx,
				OpConstants.kTimeoutMs);
		/**
		 * Phase sensor accordingly. Positive Sensor Reading should match Green
		 * (blinking) Leds on Talon
		 */
		mTalonLaunch1.setSensorPhase(false);
		mTalonLaunch2.setSensorPhase(true);

		/* Config the peak and nominal outputs */
		mTalonLaunch1.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		mTalonLaunch1.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		mTalonLaunch1.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		mTalonLaunch1.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);
		mTalonLaunch2.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		mTalonLaunch2.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		mTalonLaunch2.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		mTalonLaunch2.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		mTalonLaunch1.config_kF(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kF, OpConstants.kTimeoutMs);
		mTalonLaunch1.config_kP(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kP, OpConstants.kTimeoutMs);
		mTalonLaunch1.config_kI(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kI, OpConstants.kTimeoutMs);
		mTalonLaunch1.config_kD(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kD, OpConstants.kTimeoutMs);
		mTalonLaunch2.config_kF(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kF, OpConstants.kTimeoutMs);
		mTalonLaunch2.config_kP(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kP, OpConstants.kTimeoutMs);
		mTalonLaunch2.config_kI(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kI, OpConstants.kTimeoutMs);
		mTalonLaunch2.config_kD(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kD, OpConstants.kTimeoutMs);

		mTalonLaunch1.setNeutralMode(NeutralMode.Brake);
		mTalonLaunch2.setNeutralMode(NeutralMode.Brake);

		/*
		 * StatorCurrentLimitConfiguration statorConfig = // ENABLED LIMIT(AMP) TRIGGER
		 * THRESHOLD(AMP) TRIGGER THRESHOLD TIME(s) new
		 * StatorCurrentLimitConfiguration(true, 40, 45, 1.0);
		 * mTalonLaunch1.configStatorCurrentLimit(statorConfig);
		 * mTalonLaunch2.configStatorCurrentLimit(statorConfig);
		 */
		SupplyCurrentLimitConfiguration supplyConfig =
				// ENABLED LIMIT(AMP) TRIGGER THRESHOLD(AMP) TRIGGER THRESHOLD TIME(s)
				new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5);
		mTalonLaunch1.configSupplyCurrentLimit(supplyConfig);
		mTalonLaunch2.configSupplyCurrentLimit(supplyConfig);

		launchMode();
		hoodRetract();
		brakeOff();

		if (System.currentTimeMillis() % 100 == 0) {
			SmartDashboard.putNumber("LaunchingPercent", 0.5);
		}
	}

	public void testSpeed() {
		if(isDisabled()){
			return;
		}

		// System.out.println("testSpeed");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if(isDisabled()){
			return;
		}

		if (System.currentTimeMillis() % 100 == 0) {
			SmartDashboard.putBoolean("isHiCy", isHiCylinderSensor());
			SmartDashboard.putBoolean("isLoCy", isLoCylinderSensor());
			SmartDashboard.putBoolean("isClimbEx", isClimbExtendSensor());
			SmartDashboard.putBoolean("isClimbRt", isClimbRetractSensor());
		}
	}

	public double getLaunchMotor1Velocity() {
		if(isDisabled()){
			return 0;
		}

		return mTalonLaunch1.getSelectedSensorVelocity();
	}

	public boolean atTargetVelocity() {
		if(isDisabled()){
			return false;
		}

		return mTalonLaunch1.getSelectedSensorVelocity() >= targetVelocity_UnitsPer100ms * 0.95
				&& mTalonLaunch1.getSelectedSensorVelocity() < targetVelocity_UnitsPer100ms * 1.05;
	}

	public void enableLaunching() {
		if(isDisabled()){
			return;
		}

		// this is for Autonomous
		this.spinLauncher(0.8);
		hoodExtend();
	}

	public void spinLauncher(double launchMotorPercent_0_to_1) {
		if(isDisabled()){
			return;
		}

		targetVelocity_UnitsPer100ms = launchMotorPercent_0_to_1 * 3000.0 * 2048 / 600;
		/**
		 * Convert 500 RPM to units / 100ms. 2048(FX) 4096(SRX) Units/Rev * 500 RPM /
		 * 600 100ms/min in either direction: velocity setpoint is in units/100ms ==>
		 * 11425 is measured velocity at 80% / 0.8 = 9140/0.8 ==> 3347 is 11425 * 600 *
		 * 2048 == max speed in ticks per 100ms ==> launchPercent is 0 to 1, so 100% ==
		 * put in a value of 1.0
		 */
		/* 500 RPM in either direction */
		mTalonLaunch1.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
		mTalonLaunch2.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);

		// if(System.currentTimeMillis() % 100 == 0){
		SmartDashboard.putNumber("bdltargetvelocity", targetVelocity_UnitsPer100ms);
		SmartDashboard.putNumber("talon1Velocity", mTalonLaunch1.getSelectedSensorVelocity());
		SmartDashboard.putNumber("talon1Velocity2", mTalonLaunch2.getSelectedSensorVelocity());
		// }
	}

	public void stopLaunching() {
		if(isDisabled()){
			return;
		}

		mTalonLaunch1.set(ControlMode.PercentOutput, 0);
		mTalonLaunch2.set(ControlMode.PercentOutput, 0);
		launchMode();
	}

	// public void enableClimbing() {
	// mTalonLaunch1.set(ControlMode.PercentOutput,OpConstants.kMotorLaunchPercent);
	// mTalonLaunch2.set(ControlMode.PercentOutput,OpConstants.kMotorLaunchPercent);
	// }

	public void hoodRetract() {
		if(isDisabled()){
			return;
		}

		mLaunchHoodSolenoid.set(DoubleSolenoid.Value.kReverse);
		// isHoodExtended = false;
	}

	public void hoodExtend() {
		if(isDisabled()){
			return;
		}

		mLaunchHoodSolenoid.set(DoubleSolenoid.Value.kForward);
		// isHoodExtended = true;
	}

	public void setClimber(double percentOut) {
		if(isDisabled()){
			return;
		}

		double output = percentOut;
		// double test = Math.abs(percentOut);
		// if within deadband then set output to Zero
		// if (test < OpConstants.kJoystickDeadband) {
		// percentOut = 0;
		// }

		if (percentOut == 0) { // (Math.abs(output) < OpConstants.kClutchDeadband) {
			launchMode();
		} else {
			climbMode();
		}

		// System.out.println("climb output = " + output);
		mTalonLaunch1.set(ControlMode.PercentOutput, output * OpConstants.kClimbMaxPercent);
		mTalonLaunch2.set(ControlMode.PercentOutput, output * OpConstants.kClimbMaxPercent);
	}

	public void launchMode() {
		if(isDisabled()){
			return;
		}

		mLaunchClimbSolenoid.set(DoubleSolenoid.Value.kForward);// "clutch"
	}

	public void climbMode() {
		if(isDisabled()){
			return;
		}

		mLaunchClimbSolenoid.set(DoubleSolenoid.Value.kReverse); // "clutch"
		hoodExtend();
	}

	public void climbExtend() {
		if(isDisabled()){
			return;
		}

		mClimberSolenoid.set(DoubleSolenoid.Value.kReverse); // "raise climber arm"
	}

	public void climbRetract() {
		if(isDisabled()){
			return;
		}

		mClimberSolenoid.set(DoubleSolenoid.Value.kForward); // "lower climber arm"
	}

	public void brakeOn() {
		if(isDisabled()){
			return;
		}

		mBrakeSolenoid.set(DoubleSolenoid.Value.kForward); // brake
	}

	public void brakeOff() {
		if(isDisabled()){
			return;
		}

		mBrakeSolenoid.set(DoubleSolenoid.Value.kReverse); // brake
	}

	public boolean isHiCylinderSensor() {
		if(isDisabled()){
			return false;
		}

		return !sHiCylinder.get();
	}

	public boolean isLoCylinderSensor() {
		if(isDisabled()){
			return false;
		}

		return !sLoCylinder.get();
	}

	public boolean isClimbExtendSensor() {
		if(isDisabled()){
			return false;
		}

		return !sClimbExtend.get();
	}

	public boolean isClimbRetractSensor() {
		if(isDisabled()){
			return false;
		}

		return !sClimbRetract.get();
	}

	public void resetClimbEncoder() {
		if(isDisabled()){
			return;
		}

		mTalonLaunch1.setSelectedSensorPosition(0);
	}

	public double getClimbEncoderValue() {
		if(isDisabled()){
			return 0;
		}

		return mTalonLaunch1.getSelectedSensorPosition();
	}
}

