package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	private final DigitalInput _LeftBallSensor;
	private final DigitalInput _RightBallSensor;
	private  Boolean _LeftIntaking = false;
	private  Boolean _RightIntaking = false;
	private  Boolean _LeftEjecting = false;
	private  Boolean _RightEjecting = false;
	private boolean _SensorOverride;
	

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
			_LeftBallSensor = null;
			_RightBallSensor = null;

			return;
		}



	

		//kIntakeRetract = Bottom pneumatic, kIntakeExtend = top pneumatic
		_RightMotorIntake = new WPI_TalonFX(OpConstants.kMotorCANIntakeR, Constants.kCAN_BUS_CANIVORE);
		_RightIntakeSolenoid = new DoubleSolenoid(OpConstants.kPneumaticsCanID2, Constants.kPneumaticsType, OpConstants.kRBottomB, OpConstants.kRTopA);
		_LeftMotorIntake = new WPI_TalonFX(OpConstants.kMotorCANIntakeL, Constants.kCAN_BUS_CANIVORE);
		_LeftIntakeSolenoid = new DoubleSolenoid(OpConstants.kPneumaticsCanID1, Constants.kPneumaticsType, OpConstants.kLBottomB, OpConstants.kLTopA);

		//Defaulting the Motors
		_RightMotorIntake.configFactoryDefault();
		_LeftMotorIntake.configFactoryDefault();

		// direction
		_RightMotorIntake.setSensorPhase(false);
		_RightMotorIntake.setInverted(false);
		_LeftMotorIntake.setSensorPhase(false);
		_LeftMotorIntake.setInverted(false);
		_LeftBallSensor = new DigitalInput(5);
		_RightBallSensor = new DigitalInput(6);
		_SensorOverride = false;
	}

	@Override
	public void periodic() {
		if(isDisabled()){
			return;
		}
		// LEFT INTAKE  - assumes sensor is true when ball is blocking sensor
		if (_LeftIntaking) {
			_LeftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
			if ((_LeftBallSensor.get() && _RightBallSensor.get()) || _SensorOverride)  {
				_LeftMotorIntake.set(TalonFXControlMode.PercentOutput, OpConstants.kMotorLeftIntakeSpeed);
			} else {
				_LeftMotorIntake.set(TalonFXControlMode.PercentOutput, 0);
			}

		} else if (_LeftEjecting) {
			_LeftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
			_LeftMotorIntake.set(TalonFXControlMode.PercentOutput, -1 * OpConstants.kMotorLeftIntakeSpeed);
		} else {
			_LeftMotorIntake.set(TalonFXControlMode.PercentOutput, 0);
			_LeftIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		}

		// RIGHT INTAKE
		
		if (_RightIntaking) {
			_RightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
			if ((_LeftBallSensor.get() && _RightBallSensor.get()) || _SensorOverride)  {
				_RightMotorIntake.set(TalonFXControlMode.PercentOutput, OpConstants.kMotorRightIntakeSpeed);
			} else {
				_RightMotorIntake.set(TalonFXControlMode.PercentOutput, 0);
			}

		} else if (_RightEjecting) {
			_RightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
			_RightMotorIntake.set(TalonFXControlMode.PercentOutput, -1 * OpConstants.kMotorRightIntakeSpeed);

		} else {
			_RightMotorIntake.set(TalonFXControlMode.PercentOutput, 0);
			_RightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		}


		SmartDashboard.putBoolean("LeftBallSensor",_LeftBallSensor.get());
		SmartDashboard.putBoolean("RightBallSensor",_RightBallSensor.get());
		// SmartDashboard.putNumber("RightIntakeVelocity", _RightMotorIntake.getSelectedSensorVelocity());
		// SmartDashboard.putNumber("LeftIntakeVelocity", _LeftMotorIntake.getSelectedSensorVelocity());
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

		_RightIntaking = true;
	}

	//Extends the Right intake and spins the motor to intake
	public void extendRightEject(){
		if(isDisabled()){
			return;
		}	

		_RightEjecting = true;

	}

	//Retracts the Right intake and stops spinning the motor
	public void retractRightIntake(){
		if(isDisabled()){
			return;
		}

		_RightIntaking = false;
		_RightEjecting = false;
	}

	//Extends the Left intake and spins the motor to intake
	public void extendLeftIntake(){
		if(isDisabled()){
			return;
		}

		_LeftIntaking = true;

	}

	//Extends the Left intake and spins the motor to intake
	public void extendLeftEject(){
		if(isDisabled()){
			return;
		}

		_LeftEjecting = true;
	}

	//Retracts the Left intake and stops spinning the motor
	public void retractLeftIntake(){
		if(isDisabled()){
			return;
		}

		_LeftIntaking = false;
		_LeftEjecting = false;
	}

	public void sensorOverride(boolean sensorOverride){
		_SensorOverride = sensorOverride;
	}

}