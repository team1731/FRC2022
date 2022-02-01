/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class SmartMotionNeo extends ToggleableSubsystem {

	@Override
	protected boolean getEnabled(){
		return true;
	}

	private DoubleSolenoid mBrakeSolenoid;

	private CANSparkMax m_smart;
	private SparkMaxPIDController m_pidController;
	private RelativeEncoder m_encoder;
	public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr; 

	/**
	 * Creates a new SmartMotionNeo.
	 */
	public SmartMotionNeo() {
		if(isDisabled()){
			m_smart = null;
			return;
		}

		// initialize motor
		m_smart = new CANSparkMax(OpConstants.kMotorCANLaunch1, MotorType.kBrushless);

		/**
		 * The RestoreFactoryDefaults method can be used to reset the configuration parameters
		 * in the SPARK MAX to their factory default state. If no argument is passed, these
		 * parameters will not persist between power cycles
		 */
		m_smart.restoreFactoryDefaults();

		// initialze PID controller and encoder objects
		m_pidController = m_smart.getPIDController();
		m_encoder = m_smart.getEncoder();

		// PID coefficients
		kP = 5e-5; 
		kI = 1e-6;
		kD = 0; 
		kIz = 0; 
		kFF = 0.000156; 
		kMaxOutput = 1; 
		kMinOutput = -1;
		maxRPM = 5700;

		// Smart Motion Coefficients
		maxVel = 2000; // rpm
		maxAcc = 1500;

		// set PID coefficients
		m_pidController.setP(kP);
		m_pidController.setI(kI);
		m_pidController.setD(kD);
		m_pidController.setIZone(kIz);
		m_pidController.setFF(kFF);
		m_pidController.setOutputRange(kMinOutput, kMaxOutput);

		/**
		 * Smart Motion coefficients are set on a SparkMaxPIDController object
		 * 
		 * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
		 * the pid controller in Smart Motion mode
		 * - setSmartMotionMinOutputVelocity() will put a lower bound in
		 * RPM of the pid controller in Smart Motion mode
		 * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
		 * of the pid controller in Smart Motion mode
		 * - setSmartMotionAllowedClosedLoopError() will set the max allowed
		 * error for the pid controller in Smart Motion mode
		 */
		int smartMotionSlot = 0;
		m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
		m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
		m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
		m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

		// display PID coefficients on SmartDashboard
		SmartDashboard.putNumber("P Gain", kP);
		SmartDashboard.putNumber("I Gain", kI);
		SmartDashboard.putNumber("D Gain", kD);
		SmartDashboard.putNumber("I Zone", kIz);
		SmartDashboard.putNumber("Feed Forward", kFF);
		SmartDashboard.putNumber("Max Output", kMaxOutput);
		SmartDashboard.putNumber("Min Output", kMinOutput);

		// display Smart Motion coefficients
		SmartDashboard.putNumber("Max Velocity", maxVel);
		SmartDashboard.putNumber("Min Velocity", minVel);
		SmartDashboard.putNumber("Max Acceleration", maxAcc);
		SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
		SmartDashboard.putNumber("Set Position", 0);
		SmartDashboard.putNumber("Set Velocity", 0);

		// button to toggle between velocity and smart motion modes
		SmartDashboard.putBoolean("Mode", true);

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

	public double getSmartVelocity() {
		if(isDisabled()){
			return 0;
		}

		return m_smart.get(); // mTalonLaunch1.getSelectedSensorVelocity();
	}

	public void spinSmart(double launchMotorPercent_0_to_1) {
		if(isDisabled()){
			return;
		}

		// read PID coefficients from SmartDashboard
		double p = SmartDashboard.getNumber("P Gain", 0);
		double i = SmartDashboard.getNumber("I Gain", 0);
		double d = SmartDashboard.getNumber("D Gain", 0);
		double iz = SmartDashboard.getNumber("I Zone", 0);
		double ff = SmartDashboard.getNumber("Feed Forward", 0);
		double max = SmartDashboard.getNumber("Max Output", 0);
		double min = SmartDashboard.getNumber("Min Output", 0);
		double maxV = SmartDashboard.getNumber("Max Velocity", 0);
		double minV = SmartDashboard.getNumber("Min Velocity", 0);
		double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
		double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

		// if PID coefficients on SmartDashboard have changed, write new values to controller
		if((p != kP)) { m_pidController.setP(p); kP = p; }
		if((i != kI)) { m_pidController.setI(i); kI = i; }
		if((d != kD)) { m_pidController.setD(d); kD = d; }
		if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
		if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
		if((max != kMaxOutput) || (min != kMinOutput)) { 
			m_pidController.setOutputRange(min, max); 
			kMinOutput = min; kMaxOutput = max; 
		}
		if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
		if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
		if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
		if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

		double setPoint, processVariable;
		boolean mode = SmartDashboard.getBoolean("Mode", false);
		if(mode) {
			setPoint = SmartDashboard.getNumber("Set Velocity", 0);
			m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
			processVariable = m_encoder.getVelocity();
		} else {
			setPoint = SmartDashboard.getNumber("Set Position", 0);
			/**
			* As with other PID modes, Smart Motion is set by calling the
			* setReference method on an existing pid object and setting
			* the control type to kSmartMotion
			*/
			m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
			processVariable = m_encoder.getPosition();
		}
		
		SmartDashboard.putNumber("SetPoint", setPoint);
		SmartDashboard.putNumber("Process Variable", processVariable);
		SmartDashboard.putNumber("Output", m_smart.getAppliedOutput());
	}

	public void stopSmart() {
		if(isDisabled()){
			return;
		}

		SmartDashboard.putNumber("SetPoint", 0);
		System.out.println("SmartMagicNeo: Stopped");

		//mTalonLaunch1.set(ControlMode.PercentOutput, 0);
		//mTalonLaunch2.set(ControlMode.PercentOutput, 0);
		//launchMode();
		m_smart.set(0);
	}


	public double getSmartEncoderValue() {
		if(isDisabled()){
			return 0;
		}

		//return mTalonLaunch1.getSelectedSensorPosition();
		return m_smart.getEncoder().getPosition();
	}
}
