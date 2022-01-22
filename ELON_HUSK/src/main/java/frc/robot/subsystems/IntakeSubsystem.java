package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class IntakeSubsystem extends ToggleableSubsystem{

	@Override
	protected boolean getEnabled(){
		return true;
	}

	// private final LedStringSubsystem m_ledstring;
	private final WPI_TalonFX _RightMotorIntake;
	private final DoubleSolenoid _RightIntakeSolenoid;
	private final WPI_TalonFX _LeftMotorIntake;
	private final DoubleSolenoid _LeftIntakeSolenoid;

	/**
	 * Creates a new IntakeSubsystem.
	 * 
	 * @param m_ledstring
	 */
	public IntakeSubsystem() {
		if(isDisabled()){
			_RightMotorIntake = null;
			_RightIntakeSolenoid = null;
			_LeftMotorIntake = null;
			_LeftIntakeSolenoid = null;
			return;
		}

		_RightMotorIntake = new WPI_TalonFX(OpConstants.kMotorPWMIntake);
		_RightIntakeSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1IntakeRetract,
				OpConstants.k1IntakeExtend);
		_LeftMotorIntake = new WPI_TalonFX(OpConstants.kMotorPWMIntake);
		_LeftIntakeSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1IntakeRetract,
				OpConstants.k1IntakeExtend);
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

		_RightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
		_LeftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void active() {
		if(isDisabled()){
			return;
		}

		_RightMotorIntake.set(OpConstants.kMotorIntakeFwdSpeed);
		_LeftMotorIntake.set(OpConstants.kMotorIntakeFwdSpeed);
		// m_ledstring.option(LedOption.INTAKE);
	}

	public void inactive() {
		if(isDisabled()){
			return;
		}

		_RightMotorIntake.set(0);
		_LeftMotorIntake.set(0);
	}

	/**
	 * Enables the intake by extending solenoid & turning on motor.
	 */
	public void eject() {
		if(isDisabled()){
			return;
		}

		_RightMotorIntake.set(OpConstants.kMotorIntakeRevSpeed);
		_LeftMotorIntake.set(OpConstants.kMotorIntakeRevSpeed);
	}

	/**
	 * Enables the intake by retracting solenoid & turning off motor.
	 */
	public void retract() {
		if(isDisabled()){
			return;
		}

		_RightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		_LeftIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		_RightMotorIntake.set(0);
		_LeftMotorIntake.set(0);
	}

	//Extends the Right intake and spins the motor to intake
	public void extendRightIntake(){
		if(isDisabled()){
			return;
		}	
		_RightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
		_RightMotorIntake.set(OpConstants.kMotorIntakeFwdSpeed);
	}

	//Retracts the Right intake and stops spinning the motor
	public void retractRightIntake(){
		if(isDisabled()){
			return;
		}
		_RightIntakeSolenoid.set(DoubleSolenoid.Value.kOff);
		_RightMotorIntake.set(0);
	}

	//Extends the Left intake and spins the motor to intake
	public void extendLeftIntake(){
		if(isDisabled()){
			return;
		}
		_LeftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
		_LeftMotorIntake.set(OpConstants.kMotorIntakeFwdSpeed);
	}

	//Retracts the Left intake and stops spinning the motor
	public void retractLeftIntake(){
		if(isDisabled()){
			return;
		}
		_LeftIntakeSolenoid.set(DoubleSolenoid.Value.kOff);
		_LeftMotorIntake.set(0);
	}
}

//both sides can intake
/*Make some functions for spining a motor when a buton is pressed: both can spin at the same time
 are indepent of eachother, use commands to bind the functions to the subsystem*/