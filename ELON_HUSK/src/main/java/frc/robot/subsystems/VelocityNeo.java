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
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
//import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.OpConstants;

public class VelocityNeo extends ToggleableSubsystem {

	@Override
	protected boolean getEnabled(){
		return true;
	}

	//private DoubleSolenoid mLaunchHoodSolenoid;
	private DoubleSolenoid mBrakeSolenoid;
	///private final WPI_TalonFX mTalonLaunch1;
	//private final WPI_TalonFX mTalonLaunch2;
	private double targetVelocity_UnitsPer100ms;

	private CANSparkMax m_neo;
	private SparkMaxPIDController m_pidController;
	private RelativeEncoder m_encoder;
	public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  
	/**
	 * Creates a new VelocityNeo.
	 */
	public VelocityNeo() {
		if(isDisabled()){
			//mLaunchHoodSolenoid = null;
			//mBrakeSolenoid = null;
			///mTalonLaunch1 = null;
			//mTalonLaunch2 = null;
			m_neo = null;
			return;
		}

		// initialize motor
		m_neo = new CANSparkMax(OpConstants.kMotorCANLaunch1, MotorType.kBrushless);

		/**
		 * The RestoreFactoryDefaults method can be used to reset the configuration parameters
		 * in the SPARK MAX to their factory default state. If no argument is passed, these
		 * parameters will not persist between power cycles
		 */
		m_neo.restoreFactoryDefaults();

		/**
		 * In order to use PID functionality for a controller, a SparkMaxPIDController object
		 * is constructed by calling the getPIDController() method on an existing
		 * CANSparkMax object
		 */
		m_pidController = m_neo.getPIDController();

		// Encoder object created to display position values
		m_encoder = m_neo.getEncoder();

		// PID coefficients
		kP = 6e-5; 
		kI = 0;
		kD = 0; 
		kIz = 0; 
		kFF = 0.000015; 
		kMaxOutput = 1; 
		kMinOutput = -1;
		maxRPM = 4700;

		// set PID coefficients
		m_pidController.setP(kP);
		m_pidController.setI(kI);
		m_pidController.setD(kD);
		m_pidController.setIZone(kIz);
		m_pidController.setFF(kFF);
		m_pidController.setOutputRange(kMinOutput, kMaxOutput);

		// display PID coefficients on SmartDashboard
		SmartDashboard.putNumber("P Gain", kP);
		SmartDashboard.putNumber("I Gain", kI);
		SmartDashboard.putNumber("D Gain", kD);
		SmartDashboard.putNumber("I Zone", kIz);
		SmartDashboard.putNumber("Feed Forward", kFF);
		SmartDashboard.putNumber("Max Output", kMaxOutput);
		SmartDashboard.putNumber("Min Output", kMinOutput);
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

		//mTalonLaunch2.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
		if (System.currentTimeMillis() % 100 == 0) {
		}
	}

	public double getNeoVelocity() {
		if(isDisabled()){
			return 0;
		}

		return m_neo.get(); // mTalonLaunch1.getSelectedSensorVelocity();
	}

	public void spinLauncher(double launchMotorPercent_0_to_1) {
		if(isDisabled()){
			return;
		}

		// double max = 0.71;
		// double min = 0.176;
		// launchMotorPercent_0_to_1 = (launchMotorPercent_0_to_1 - min) / (max - min);
		// launchMotorPercent_0_to_1 = Math.max(0, Math.min(launchMotorPercent_0_to_1, 1));
		// launchMotorPercent_0_to_1 *= -1;

		//targetVelocity_UnitsPer100ms = launchMotorPercent_0_to_1;
			/**
			 * Convert 2000 RPM to units / 100ms.
			 * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
		targetVelocity_UnitsPer100ms = launchMotorPercent_0_to_1 * 2000.0 * 2048.0 / 300.0;
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
		//mTalonLaunch1.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
		//mTalonLaunch1.set(TalonFXControlMode.PercentOutput, launchMotorPercent_0_to_1);

		// if(System.currentTimeMillis() % 100 == 0){
		//SmartDashboard.putNumber("mTalonLaunch1Inp",targetVelocity_UnitsPer100ms);
		//SmartDashboard.putNumber("talon1Velocity", mTalonLaunch1.getSelectedSensorVelocity());
		//SmartDashboard.putNumber("mTalonLaunch1Vel", mTalonLaunch1.getSelectedSensorVelocity());
		double setPoint = launchMotorPercent_0_to_1*maxRPM;
		m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
		
		SmartDashboard.putNumber("SetPoint", setPoint);
		SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
	}

	public void stopLaunching() {
		if(isDisabled()){
			return;
		}

		SmartDashboard.putNumber("SetPoint", 0);
		System.out.println("VelocityNeo: Stopped");

		//mTalonLaunch1.set(ControlMode.PercentOutput, 0);
		//mTalonLaunch2.set(ControlMode.PercentOutput, 0);
		//launchMode();
		m_neo.set(0);
	}

	public void resetNeoEncoder() {
		if(isDisabled()){
			return;
		}

		//mTalonLaunch1.setSelectedSensorPosition(0);
		///mTalonLaunch2.setSelectedSensorPosition(0);
	}

	public double getNeoEncoderValue() {
		if(isDisabled()){
			return 0;
		}

		//return mTalonLaunch1.getSelectedSensorPosition();
		return m_neo.getEncoder().getPosition();
	}
}

