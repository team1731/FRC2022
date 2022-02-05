package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * The way intake works is there is a motor for left intake, a motor for right intake, and a conveyor motor which takes the balls from intake and pulls
 * them into the shooter.  When the left button is toggled, the left intake motor and conveyor activate; when the right button is toggled, the right intake
 * motor and conveyor activates; when both buttons are toggled, both intake motors and the conveyor activate.
 * TODO: 1. Make sure the logic for having both on then turning off one works
 * CAN Id 6 = Right Motor Intake
 * Can Id 7 = Conveyor
 * Can Id 8 = Left Motor Intake
 */
public class IntakeSubsystem extends ToggleableSubsystem{

	@Override
	protected boolean getEnabled(){
		return false;
	}

	private final WPI_TalonFX _RightMotorIntake;
	// private final DoubleSolenoid _RightIntakeSolenoid;
	private final WPI_TalonFX _LeftMotorIntake;
	// private final DoubleSolenoid _LeftIntakeSolenoid;
	private final WPI_TalonFX _ConveyorMotorIntake;

	private boolean _LeftEnabled = false;
	private boolean _RightEnabled = false;

	/**
	 * Creates a new IntakeSubsystem.
	 * 
	 * @param m_ledstring
	 */
	public IntakeSubsystem() {
		if(isDisabled()){
			_RightMotorIntake = null;
			// _RightIntakeSolenoid = null;
			_LeftMotorIntake = null;
			// _LeftIntakeSolenoid = null;
			_ConveyorMotorIntake = null;
			return;
		}

		_RightMotorIntake = new WPI_TalonFX(OpConstants.kMotorCANIntake1);
		// _RightIntakeSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1IntakeRetract,
				// OpConstants.k1IntakeExtend);
		_LeftMotorIntake = new WPI_TalonFX(OpConstants.kMotorCANIntake2);
		// _LeftIntakeSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1IntakeRetract,
				// OpConstants.k1IntakeExtend);
		_ConveyorMotorIntake = new WPI_TalonFX(OpConstants.kMotorCANIntakeConveyor);

		//Defaulting the Motors
		_RightMotorIntake.configFactoryDefault();
		_LeftMotorIntake.configFactoryDefault();
		_ConveyorMotorIntake.configFactoryDefault();

		_RightMotorIntake.configNeutralDeadband(0.001);
		_LeftMotorIntake.configNeutralDeadband(0.001);
		_ConveyorMotorIntake.configNeutralDeadband(0.001);

		_RightMotorIntake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,OpConstants.kPIDLoopIdx,
		 OpConstants.kTimeoutMs);
		_LeftMotorIntake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,OpConstants.kPIDLoopIdx,
		OpConstants.kTimeoutMs);
		_ConveyorMotorIntake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,OpConstants.kPIDLoopIdx,
		OpConstants.kTimeoutMs);

		_RightMotorIntake.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		_RightMotorIntake.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		_RightMotorIntake.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		_RightMotorIntake.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);

		_LeftMotorIntake.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		_LeftMotorIntake.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		_LeftMotorIntake.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		_LeftMotorIntake.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);
		
		_ConveyorMotorIntake.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		_ConveyorMotorIntake.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		_ConveyorMotorIntake.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		_ConveyorMotorIntake.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);

		_RightMotorIntake.config_kF(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kF, OpConstants.kTimeoutMs);
		_RightMotorIntake.config_kP(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kP, OpConstants.kTimeoutMs);
		_RightMotorIntake.config_kI(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kI, OpConstants.kTimeoutMs);
		_RightMotorIntake.config_kD(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kD, OpConstants.kTimeoutMs);

		_LeftMotorIntake.config_kF(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kF, OpConstants.kTimeoutMs);
		_LeftMotorIntake.config_kP(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kP, OpConstants.kTimeoutMs);
		_LeftMotorIntake.config_kI(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kI, OpConstants.kTimeoutMs);
		_LeftMotorIntake.config_kD(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kD, OpConstants.kTimeoutMs);

		_ConveyorMotorIntake.config_kF(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kF, OpConstants.kTimeoutMs);
		_ConveyorMotorIntake.config_kP(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kP, OpConstants.kTimeoutMs);
		_ConveyorMotorIntake.config_kI(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kI, OpConstants.kTimeoutMs);
		_ConveyorMotorIntake.config_kD(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kD, OpConstants.kTimeoutMs);
		
		_RightMotorIntake.setNeutralMode(NeutralMode.Coast);
		_LeftMotorIntake.setNeutralMode(NeutralMode.Coast);
		_ConveyorMotorIntake.setNeutralMode(NeutralMode.Coast);
	}

	@Override
	public void periodic() {
		if(isDisabled()){
			return;
		}

		SmartDashboard.putNumber("RightIntakeVelocity", _RightMotorIntake.getSelectedSensorVelocity());
		SmartDashboard.putNumber("LeftIntakeVelocity", _LeftMotorIntake.getSelectedSensorVelocity());
		SmartDashboard.putNumber("ConveyorVelocity", _ConveyorMotorIntake.getSelectedSensorVelocity());
		// This method will be called once per scheduler run
	}

	/**
	 * Enables the intake by extending solenoid & turning on motor.
	 */
	public void extend() {
		if(isDisabled()){
			return;
		}

		// _RightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
		// _LeftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void active() {
		if(isDisabled()){
			return;
		}

		//_RightMotorIntake.set(OpConstants.kMotorIntakeFwdSpeed);
		//_LeftMotorIntake.set(OpConstants.kMotorIntakeFwdSpeed);
		// m_ledstring.option(LedOption.INTAKE);
	}

	public void inactive() {
		if(isDisabled()){
			return;
		}

		//_RightMotorIntake.set(0);
		//_LeftMotorIntake.set(0);
	}

	/**
	 * Enables the intake by extending solenoid & turning on motor.
	 */
	public void eject() {
		if(isDisabled()){
			return;
		}

		//_RightMotorIntake.set(OpConstants.kMotorIntakeRevSpeed);
		//_LeftMotorIntake.set(OpConstants.kMotorIntakeRevSpeed);
	}

	/**
	 * Enables the intake by retracting solenoid & turning off motor.
	 */
	public void retract() {
		if(isDisabled()){
			return;
		}

		// _RightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		// _LeftIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		//_RightMotorIntake.set(0);
		//_LeftMotorIntake.set(0);
	}

	//Extends the Right intake and spins the motor to intake
	public void extendRightIntake(){
		if(isDisabled()){
			return;
		}	

		_RightEnabled = true;

		// _RightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
		if(_RightEnabled == true || _LeftEnabled == true){
			_RightMotorIntake.set(TalonFXControlMode.Velocity, OpConstants.kMotorIntakeFwdSpeed * 2000.0 * 2048.0 / 600);
			activateConveyor();
		}
		//_RightMotorIntake.set(TalonFXControlMode.PercentOutput, OpConstants.kMotorIntakeFwdSpeed * 0.2);
	}

	//Retracts the Right intake and stops spinning the motor
	public void retractRightIntake(){
		if(isDisabled()){
			return;
		}
		// _RightIntakeSolenoid.set(DoubleSolenoid.Value.kOff);
		_RightMotorIntake.set(0);
		_RightEnabled = false;
	}

	//Extends the Left intake and spins the motor to intake
	public void extendLeftIntake(){
		if(isDisabled()){
			return;
		}

		_LeftEnabled = true;

		// _LeftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
		if(_LeftEnabled == true || _RightEnabled == true){
			_LeftMotorIntake.set(TalonFXControlMode.Velocity, OpConstants.kMotorIntakeFwdSpeed * 2000.0 * 2048.0 / 600.0);
			activateConveyor();
		}		
		//_LeftMotorIntake.set(TalonFXControlMode.PercentOutput, OpConstants.kMotorIntakeFwdSpeed * 0.2 );
		_LeftEnabled = true;
	}

	//Retracts the Left intake and stops spinning the motor
	public void retractLeftIntake(){
		if(isDisabled()){
			return;
		}
		// _LeftIntakeSolenoid.set(DoubleSolenoid.Value.kOff);
		_LeftMotorIntake.set(0);
		_LeftEnabled = false;
	}

	//Enables the conveyor motor
	public void activateConveyor(){
		if(isDisabled()){
			return;
		}
		_ConveyorMotorIntake.set(TalonFXControlMode.Velocity, OpConstants.kMotorCANIntakeConveyor * 2000.0 * 2048.0 / 600.0);
		//_ConveyorMotorIntake.set(TalonFXControlMode.PercentOutput, OpConstants.kMotorCANIntakeConveyor * 0.2);
	}

	//Disables the conveyor motor
	public void deActivateConveyor(){
		if(isDisabled()){
			return;
		}
		_ConveyorMotorIntake.set(0);
	}
}