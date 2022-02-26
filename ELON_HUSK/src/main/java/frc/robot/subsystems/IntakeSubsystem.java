package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
//import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * The way intake works is there is a motor for left intake, a motor for right intake, and a conveyor motor which takes the balls from intake and pulls
 * them into the shooter.  When the left button is toggled, the left intake motor and conveyor activate; when the right button is toggled, the right intake
 * motor and conveyor activates; when both buttons are toggled, both intake motors and the conveyor activate.
 * CAN Id 8 = Right Motor Intake
 * Can Id 7 = Left Motor Intake
 */
public class IntakeSubsystem extends ToggleableSubsystem{

	@Override
	protected boolean getEnabled(){
		return true;
	}

	private final WPI_TalonFX _RightMotorIntake;
	private final DoubleSolenoid _RightIntakeSolenoid;
	private final WPI_TalonFX _LeftMotorIntake;
	private final DoubleSolenoid _LeftIntakeSolenoid;

	private boolean _LeftEnabled = true;
	private boolean _RightEnabled = true;

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

		//kIntakeRetract = Bottom pneumatic, kIntakeExtend = top pneumatic
		_RightMotorIntake = new WPI_TalonFX(OpConstants.kMotorCANIntakeR, Constants.kCAN_BUS_CANIVORE);
		_RightIntakeSolenoid = new DoubleSolenoid(OpConstants.kPneumaticsCanID, Constants.kPneumaticsType, OpConstants.kRBottomB, OpConstants.kRTopA);
		_LeftMotorIntake = new WPI_TalonFX(OpConstants.kMotorCANIntakeL, Constants.kCAN_BUS_CANIVORE);
		_LeftIntakeSolenoid = new DoubleSolenoid(OpConstants.kPneumaticsCanID, Constants.kPneumaticsType, OpConstants.kLBottomB, OpConstants.kLTopA);

		//Defaulting the Motors
		_RightMotorIntake.configFactoryDefault();
		_LeftMotorIntake.configFactoryDefault();

		// direction
		_RightMotorIntake.setSensorPhase(false);
		_RightMotorIntake.setInverted(false);
		_LeftMotorIntake.setSensorPhase(false);
		_LeftMotorIntake.setInverted(false);

		SmartDashboard.putBoolean("RightIntakeOn",_RightEnabled);
		SmartDashboard.putBoolean("LeftIntakeOn", _LeftEnabled);
	}

	@Override
	public void periodic() {
		if(isDisabled()){
			return;
		}

		// SmartDashboard.putNumber("RightIntakeVelocity", _RightMotorIntake.getSelectedSensorVelocity());
		// SmartDashboard.putNumber("LeftIntakeVelocity", _LeftMotorIntake.getSelectedSensorVelocity());

		// SmartDashboard.putBoolean("RightIntakeOn",_RightEnabled);
		// SmartDashboard.putBoolean("LeftIntakeOn", _LeftEnabled);

		// This method will be called once per scheduler run
	}

	/**
	 * Enables the intake by retracting solenoid & turning off motor.
	 */
	public void retract() {
		if(isDisabled()){
			return;
		}

		retractRightIntake();
		retractLeftIntake();
	}

	//Extends the Right intake and spins the motor to intake
	public void extendRightIntake(){
		if(isDisabled()){
			return;
		}	

		_RightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
		//_RightMotorIntake.set(TalonFXControlMode.Velocity, OpConstants.kMotorIntakeFwdSpeed * 2000.0 * 2048.0 / 600);
		_RightMotorIntake.set(TalonFXControlMode.PercentOutput, OpConstants.kMotorRightIntakeSpeed);

		_RightEnabled = true;
	}

	//Retracts the Right intake and stops spinning the motor
	public void retractRightIntake(){
		if(isDisabled()){
			return;
		}

		_RightMotorIntake.set(TalonFXControlMode.PercentOutput, 0);
		_RightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);

		_RightEnabled = false;
	}

	//Extends the Left intake and spins the motor to intake
	public void extendLeftIntake(){
		if(isDisabled()){
			return;
		}

		_LeftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
		_LeftMotorIntake.set(TalonFXControlMode.PercentOutput, OpConstants.kMotorLeftIntakeSpeed);
		
		_LeftEnabled = true;
	}

	//Retracts the Left intake and stops spinning the motor
	public void retractLeftIntake(){
		if(isDisabled()){
			return;
		}

		_LeftMotorIntake.set(TalonFXControlMode.PercentOutput, 0);
		_LeftIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		_LeftEnabled = false;
	}

}