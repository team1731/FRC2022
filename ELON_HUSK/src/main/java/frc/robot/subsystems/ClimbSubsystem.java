package frc.robot.subsystems;


import java.util.Arrays;
import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.IRSensor;
import frc.robot.Constants.OpConstants;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends ToggleableSubsystem {

	//#region ToggleableSubsystem
	@Override
	protected boolean getEnabled(){
		return m_Enabled;
	}

	private boolean m_Enabled = true;
	//#endregion

	private State _currentState = State.READY;
	private InputDirection _inputDirection = InputDirection.NEUTRAL;

	private final DoubleSolenoid _extender;
	private final DoubleSolenoid _grabberNorthFront;
	private final DoubleSolenoid _grabberNorthBack;
	private final DoubleSolenoid _grabberSouthFront;
	private final DoubleSolenoid _grabberSouthBack;

	private final CANSparkMax _swingerMasterMotor;
	private final CANSparkMax _swingerSlaveMotor;

	private final IRSensor _northSensor;
	private final IRSensor _southSensor;

	private final SparkMaxPIDController _pidMasterController;
	private final RelativeEncoder _encoderMaster;
	private final SparkMaxPIDController _pidSlaveController;

	private double _timer = System.currentTimeMillis();
	private int _climbCount = 0;
	private boolean _sensorOverride = false;

	//#region Enums

	public enum State {
		READY(0),                // Extenders down, north grabbers open, south grabbers closed
		EXTEND(1),               // Extenders up, north grabber one half closed, south grabbers closed
		GRAB_FIRST_BAR(2),       // Extenders up, north grabber closed, south grabber closed

		SWING_TO_SECOND_BAR(3),  // Extenders up, north grabber closed, south grabber closed, swinger motors spinning+
		GRAB_SECOND_BAR(4),      // Extenders up, north grabber closed, south grabber closed
		RELEASE_FIRST_BAR(5),    // Extenders up, north grabber open, south grabber closed

		SWING_TO_THIRD_BAR(6),
		GRAB_THIRD_BAR(7),
		RELEASE_SECOND_BAR(8),

		HANG(9);                 // Extenders up, north grabber open, south grabber closed

		private final int _value;

		State(int value){
			_value = value;
		}

		public State previous(){
			Optional<State> previousState = State.valueOf(this._value-1);
			if(previousState.isPresent()){
				return previousState.get();
			}
			
			return this;
		}

		public State next(){
			Optional<State> nextState = State.valueOf(this._value+1);
			if(nextState.isPresent()){
				return nextState.get();
			}

			return this;
		}

		public static Optional<State> valueOf(int value){
			return Arrays.stream(values())
				.filter(state -> state._value == value)
				.findFirst();
		}
	}

	public enum InputDirection {
		UP,
		NEUTRAL,
		
		// UP(1),
		// NEUTRAL(0),
		// DOWN(-1);

		// public final int value;

		// InputDirection(int value){
		// 	this.value = value;
		// }
	}

	private enum GrabberHalf {
		FRONT,
		BACK,
		BOTH;
	}

	//#endregion

	public ClimbSubsystem() {
		if(isDisabled()){ 
			_extender = null;
			_grabberNorthFront = null;
			_grabberNorthBack = null;
			_grabberSouthFront = null;
			_grabberSouthBack = null;
			_swingerMasterMotor = null;
			_swingerSlaveMotor = null;
			_northSensor = null;
			_southSensor = null;
			_pidMasterController = null;
			_encoderMaster = null;
			_pidSlaveController = null;
			return;
		}

		_extender = new DoubleSolenoid(OpConstants.kPneumaticsCanID, Constants.kPneumaticsType, OpConstants.kExtenderUpID, OpConstants.kExtenderDownID);
		_grabberNorthFront = new DoubleSolenoid(OpConstants.kPneumaticsCanID, Constants.kPneumaticsType, OpConstants.kGrabberNorthFrontCloseID, OpConstants.kGrabberNorthFrontOpenID);
		_grabberNorthBack =  new DoubleSolenoid(OpConstants.kPneumaticsCanID, Constants.kPneumaticsType, OpConstants.kGrabberNorthBackCloseID, OpConstants.kGrabberNorthBackOpenID);
		_grabberSouthFront = new DoubleSolenoid(OpConstants.kPneumaticsCanID, Constants.kPneumaticsType, OpConstants.kGrabberSouthFrontCloseID, OpConstants.kGrabberSouthFrontOpenID);
		_grabberSouthBack =  new DoubleSolenoid(OpConstants.kPneumaticsCanID, Constants.kPneumaticsType, OpConstants.kGrabberSouthBackCloseID, OpConstants.kGrabberSouthBackOpenID);

		if(_extender == null){
			m_Enabled = false;
			System.err.println("CLIMB FATAL: UNABLE TO INITIALIZE EXTENDER DOUBLESOLENOID!!!");
			System.err.println("ClimbSubsystem will be DISABLED for mechanical safety until robot code restarts!");
		}

		_swingerMasterMotor = new CANSparkMax(OpConstants.kLeftSwingerMotorID, MotorType.kBrushless);
		_swingerSlaveMotor = new CANSparkMax(OpConstants.kRightSwingerMotorID, MotorType.kBrushless);

		_northSensor = new IRSensor(OpConstants.kNorthSensorID);
		_southSensor = new IRSensor(OpConstants.kSouthSensorID);

		/**
		 * Configuring Defaults for Master/Slave motors
		 */
		_swingerMasterMotor.restoreFactoryDefaults();
        _swingerMasterMotor.setInverted(true);

		_pidMasterController = _swingerMasterMotor.getPIDController();
		_encoderMaster = _swingerMasterMotor.getEncoder();

		// set PID coefficients
		_pidMasterController.setP(ClimbConstants.kP);
		_pidMasterController.setI(ClimbConstants.kI);
		_pidMasterController.setD(ClimbConstants.kD);
		_pidMasterController.setIZone(ClimbConstants.kIz);
		_pidMasterController.setFF(ClimbConstants.kFF);
		_pidMasterController.setOutputRange(ClimbConstants.kMinOutput, ClimbConstants.kMaxOutput);

    	_pidMasterController.setSmartMotionMaxVelocity(ClimbConstants.maxVel, ClimbConstants.smartMotionSlot);
    	_pidMasterController.setSmartMotionMinOutputVelocity(ClimbConstants.minVel, ClimbConstants.smartMotionSlot);
    	_pidMasterController.setSmartMotionMaxAccel(ClimbConstants.maxAcc, ClimbConstants.smartMotionSlot);
    	_pidMasterController.setSmartMotionAllowedClosedLoopError(ClimbConstants.allowedErr, ClimbConstants.smartMotionSlot);
 
		//Slave defaults
		_swingerSlaveMotor.restoreFactoryDefaults();

		

		_pidSlaveController = _swingerSlaveMotor.getPIDController();

		// set PID coefficients
		_pidSlaveController.setP(ClimbConstants.kP);
		_pidSlaveController.setI(ClimbConstants.kI);
		_pidSlaveController.setD(ClimbConstants.kD);
		_pidSlaveController.setIZone(ClimbConstants.kIz);
		_pidSlaveController.setFF(ClimbConstants.kFF);
		_pidSlaveController.setOutputRange(ClimbConstants.kMinOutput, ClimbConstants.kMaxOutput);

    	_pidSlaveController.setSmartMotionMaxVelocity(ClimbConstants.maxVel, ClimbConstants.smartMotionSlot);
    	_pidSlaveController.setSmartMotionMinOutputVelocity(ClimbConstants.minVel, ClimbConstants.smartMotionSlot);
    	_pidSlaveController.setSmartMotionMaxAccel(ClimbConstants.maxAcc, ClimbConstants.smartMotionSlot);
    	_pidSlaveController.setSmartMotionAllowedClosedLoopError(ClimbConstants.allowedErr, ClimbConstants.smartMotionSlot);
		_swingerSlaveMotor.follow(_swingerMasterMotor, true);
		updateSmartDashboard();
		handleReady();
	}

	public State getState(){
		if(isDisabled()) return State.READY;

		return _currentState;
	}

	public void setInputDirection(InputDirection value){
		if(isDisabled()) return; 
		
		_inputDirection = value;
	}

	public boolean getSensorOveride(){
		if(isDisabled()) return false;

		return _sensorOverride;
	}

	public void setSensorOverride(boolean value){
		if(isDisabled()) return;

		_sensorOverride = value;
	}

	//#region Controls

	private void setExtenders(boolean up){
		_extender.set(up ? Value.kForward : Value.kReverse);
	}

	private void setNorthGrabber(GrabberHalf grabberHalf, boolean closed){
		if(grabberHalf == GrabberHalf.FRONT || grabberHalf == GrabberHalf.BOTH) {
			_grabberNorthFront.set(closed ? Value.kForward : Value.kReverse);
		}
		if(grabberHalf == GrabberHalf.BACK || grabberHalf == GrabberHalf.BOTH) {
			_grabberNorthBack.set(closed ? Value.kForward : Value.kReverse);
		}
	}

	private void setNorthGrabbers(boolean closed){
		setNorthGrabber(GrabberHalf.BOTH, closed);
	} 

	private void setSouthGrabber(GrabberHalf grabberHalf, boolean closed){
		if(grabberHalf == GrabberHalf.FRONT || grabberHalf == GrabberHalf.BOTH) {
			_grabberSouthFront.set(closed ? Value.kForward : Value.kReverse);
		}
		if(grabberHalf == GrabberHalf.BACK || grabberHalf == GrabberHalf.BOTH) {
			_grabberSouthBack.set(closed ? Value.kForward : Value.kReverse);
		}
	}

	private void setSouthGrabbers(boolean closed){
		setSouthGrabber(GrabberHalf.BOTH, closed);
	}

	private void startSwing(){
		_pidMasterController.setReference(
			ClimbConstants.kFwdSteps, // * _inputDirection.value,
			CANSparkMax.ControlType.kSmartMotion
		);
	}

	private void stopSwing(){
		_swingerMasterMotor.set(0);
		_encoderMaster.setPosition(0); // not sure if this is needed
	}

	//Function for properly releasing the previous par before hanging on the current bar
	//Removes 10 ticks from the current revolution count to let the climber hang
	private void releaseSwing(){
		_pidMasterController.setReference(
			ClimbConstants.kBckSteps,
			CANSparkMax.ControlType.kSmartMotion
		);
	}

	//#endregion

	//#region State Handlers

	private boolean handleReady(){
		// Extenders down, north grabbers open, south grabbers closed
		setExtenders(false);
		setNorthGrabbers(false);
		setSouthGrabbers(true);
		stopSwing();
		return _inputDirection == InputDirection.UP;
	}

	private boolean handleExtend(){
		// Extenders up, north grabber one half closed, south grabbers closed
		setExtenders(true);
		setNorthGrabber(GrabberHalf.FRONT, true);
		setSouthGrabbers(true);
		
		return _sensorOverride || (_northSensor != null && _northSensor.isTriggered());
	}

	private boolean handleGrabFirstBar(){
		// Extenders up, north grabber closed, south grabber closed
		setExtenders(true);
		setNorthGrabbers(true);

		return Timer.getFPGATimestamp() - _timer >= 1;
	}

	private boolean handleSwingToSecondBar(){
		// Extenders up, north grabber closed, south grabber half open, swinger motors spinning+
		setExtenders(true);
		startSwing();
		setSouthGrabber(GrabberHalf.BACK, false);

		return _sensorOverride || (_southSensor != null && _southSensor.isTriggered());
	}

	private boolean handleGrabSecondBar(){
		// Extenders up, north grabber closed, south grabber closed
		setExtenders(true);
		setSouthGrabbers(true);
		stopSwing();

		return Timer.getFPGATimestamp() - _timer >= 1;
	}

	private boolean handleReleaseFirstBar(){
		// Extenders up, north grabber open, south grabber closed
		setExtenders(true);
		setNorthGrabbers(false);
		releaseSwing();

		return Timer.getFPGATimestamp() - _timer >= 1;
	}

	private boolean handleSwingToThirdBar(){
		setExtenders(true);
		startSwing();
		setNorthGrabber(GrabberHalf.FRONT, false);

		return _sensorOverride || (_northSensor != null && _northSensor.isTriggered());
	}

	private boolean handleGrabThirdBar(){
		setExtenders(true);
		setNorthGrabbers(true);
		stopSwing();

		return Timer.getFPGATimestamp() - _timer >= 1;
	}

	private boolean handleReleaseSecondBar(){
		setExtenders(true);
		setSouthGrabbers(false);
		releaseSwing();

		return Timer.getFPGATimestamp() - _timer >= 1;
	}

	private void handleHang(){
		// Extenders up, north grabber open, south grabber closed
		setExtenders(true);
		setNorthGrabbers(false);
		setSouthGrabbers(true);
		_pidMasterController.setReference(
			ClimbConstants.kFwdSteps / 2,
			CANSparkMax.ControlType.kSmartMotion
		);
	}

	//#endregion

	private void updateSmartDashboard(){
		SmartDashboard.putString("climb_State", _currentState.name());
		SmartDashboard.putString("climb_iDir", _inputDirection.name());
		SmartDashboard.putBoolean("climb_GrbNF Closed", _grabberNorthFront.get() == Value.kForward);
		SmartDashboard.putBoolean("climb_GrbNB Closed", _grabberNorthBack.get() == Value.kForward);
		SmartDashboard.putBoolean("climb_GrbSF Closed", _grabberSouthFront.get() == Value.kForward);
		SmartDashboard.putBoolean("climb_GrbSB Closed", _grabberSouthBack.get() == Value.kForward);
		SmartDashboard.putBoolean("climb_Extender", _extender.get() == Value.kForward);
		SmartDashboard.putNumber("climb_encPos", _encoderMaster.getPosition());
		SmartDashboard.putNumber("climb_North Sensor Voltage", _northSensor != null ? _northSensor.getVoltage() : 0);
		SmartDashboard.putBoolean("climb_North Sensor Triggered", _northSensor != null ? _northSensor.isTriggered() : false);
		SmartDashboard.putNumber("climb_South Sensor Voltage", _southSensor != null ? _southSensor.getVoltage() : 0);
		SmartDashboard.putBoolean("climb_South Sensor Triggered", _southSensor != null ? _southSensor.isTriggered() : false);
		SmartDashboard.putBoolean("climb_Sensor Override", _sensorOverride);
		SmartDashboard.putNumber("climb_Count", _climbCount);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if(isDisabled()) return;

		updateSmartDashboard();

		if(_inputDirection == InputDirection.NEUTRAL){
			_swingerMasterMotor.set(0);
			_timer = Timer.getFPGATimestamp();
			return;
		}

		/*
		READY(0),                // Extenders down, north grabbers open, south grabbers closed
		EXTEND(1),               // Extenders up, north grabber one half closed, south grabbers closed
		GRAB_BAR(2),             // Extenders up, north grabber closed, south grabber closed

		SWING_TO_NEXT_BAR(3),    // Extenders up, north grabber closed, south grabber closed, swinger motors spinning+
		GRAB_NEXT_BAR(4),        // Extenders up, north grabber closed, south grabber closed
		RELEASE_PREVIOUS_BAR(5); // Extenders up, north grabber open, south grabber closed
		*/
		boolean transition = false;
		switch(_currentState){
			case READY:
				transition = handleReady();
				break;
			case EXTEND:
				transition = handleExtend();
				break;
			case GRAB_FIRST_BAR:
				transition = handleGrabFirstBar();
				break;

			//Loop these to climb all the way up to the top
			case SWING_TO_SECOND_BAR:
				transition = handleSwingToSecondBar();
				break;
			case GRAB_SECOND_BAR:
				transition = handleGrabSecondBar();
				break;
			case RELEASE_FIRST_BAR:
				transition = handleReleaseFirstBar();
				break;

			case SWING_TO_THIRD_BAR:
				transition = handleSwingToThirdBar();
				break;
			case GRAB_THIRD_BAR:
				transition = handleGrabThirdBar();
				break;
			case RELEASE_SECOND_BAR:
				transition = handleReleaseSecondBar();
				break;

			case HANG:
				handleHang();
				break;
		}

		if(transition){
			_timer = Timer.getFPGATimestamp();
			_currentState = _currentState.next();
			_sensorOverride = false;
			// _currentState = _inputDirection == InputDirection.UP ? _currentState.next()
			// 	: _currentState.previous();
		}
	}

	
}
