package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class IntakeSubsystem extends ToggleableSubsystem {

	@Override
	protected boolean getEnabled(){
		return true;
	}

	// private final LedStringSubsystem m_ledstring;
	private final PWMTalonFX mTalonIntake;
	private final DoubleSolenoid mIntakeSolenoid;
	private String mTalonState;

	/**
	 * Creates a new IntakeSubsystem.
	 * 
	 * @param m_ledstring
	 */
	public IntakeSubsystem() {
		if(isDisabled()){
			mTalonIntake = null;
			mIntakeSolenoid = null;
			return;
		}

		mTalonIntake = new PWMTalonFX(OpConstants.kMotorPWMIntake);
		mIntakeSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1IntakeRetract,
				OpConstants.k1IntakeExtend);
		mTalonState = "Off";
	}

	@Override
	public void periodic() {
		if(isDisabled()){
			return;
		}

		// This method will be called once per scheduler run
	}

	/**
	 * Enables the intake by extending solenoid & turning on motor.
	 */
	public void extend() {
		if(isDisabled()){
			return;
		}

		mIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
		mTalonState = "Intake Extend";
	}

	public void active() {
		if(isDisabled()){
			return;
		}

		mTalonIntake.set(OpConstants.kMotorIntakeFwdSpeed);
		mTalonState = "Intake Fwd";
		// m_ledstring.option(LedOption.INTAKE);
	}

	public void inactive() {
		if(isDisabled()){
			return;
		}

		mTalonIntake.set(0);
		mTalonState = "Intake Stop";
	}

	/**
	 * Enables the intake by extending solenoid & turning on motor.
	 */
	public void eject() {
		if(isDisabled()){
			return;
		}

		mTalonIntake.set(OpConstants.kMotorIntakeRevSpeed);
		mTalonState = "Intake Rev";
	}

	/**
	 * Enables the intake by retracting solenoid & turning off motor.
	 */
	public void retract() {
		if(isDisabled()){
			return;
		}

		mIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		mTalonIntake.set(0);
		mTalonState = "Intake Retracted/Off";
	}

	public String getIntakeState() {
		if(isDisabled()){
			return "N/A";
		}

		return (mTalonState);
	}
}
