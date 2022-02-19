package frc.robot.subsystems;


import java.util.Arrays;
import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.IRSensor;
import frc.robot.Constants.OpConstants;
import frc.robot.Constants.CanSparkMaxConstants;

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

	private final Solenoid _leftExtender;
	private final Solenoid _rightExtender;
	private final Solenoid _grabberNorth1;
	private final Solenoid _grabberNorth2;
	private final Solenoid _grabberSouth1;
	private final Solenoid _grabberSouth2;

	private final CANSparkMax _swingerMasterMotor;
	private final CANSparkMax _swingerSlaveMotor;

	private final IRSensor _northSensor;
	private final IRSensor _southSensor;

	private final SparkMaxPIDController _pidMasterController;
	private final RelativeEncoder _encoderMaster;
	private final SparkMaxPIDController _pidSlaveController;
	private final RelativeEncoder _encoderSlave;

	private double _timer = System.currentTimeMillis();
	private int _climbCount = 0;
	private boolean _sensorOverride = false;

	//#region Enums

	public enum State {
		READY(0),                // Extenders down, north grabbers open, south grabbers closed
		EXTEND(1),               // Extenders up, north grabber one half closed, south grabbers closed
		GRAB_BAR(2),             // Extenders up, north grabber closed, south grabber closed

		SWING_TO_NEXT_BAR(3),    // Extenders up, north grabber closed, south grabber closed, swinger motors spinning+
		GRAB_NEXT_BAR(4),        // Extenders up, north grabber closed, south grabber closed
		RELEASE_PREVIOUS_BAR(5), // Extenders up, north grabber open, south grabber closed

		HANGING(6);              // Extenders up, north grabber open, south grabber closed

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
		UP(1),
		NEUTRAL(0),
		DOWN(-1);

		public final int value;

		InputDirection(int value){
			this.value = value;
		}
	}

	private enum GrabberHalf {
		EAST(1),
		WEST(2),
		BOTH(3);

		private final int value;

		GrabberHalf(int value){
			this.value = value;
		}
	}

	//#endregion

	public ClimbSubsystem() {
		if(isDisabled()){ 
			_leftExtender = null;
			_rightExtender = null;
			_grabberNorth1 = null;
			_grabberNorth2 = null;
			_grabberSouth1 = null;
			_grabberSouth2 = null;
			_swingerMasterMotor = null;
			_swingerSlaveMotor = null;
			_northSensor = null;
			_southSensor = null;
			_pidMasterController = null;
			_encoderMaster = null;
			_pidSlaveController = null;
			_encoderSlave = null;
			return;
		}

		_leftExtender  = new Solenoid(Constants.kPneumaticsType, OpConstants.kLeftExtenderID);
		_rightExtender = new Solenoid(Constants.kPneumaticsType, OpConstants.kRightExtenderID);
		_grabberNorth1 = new Solenoid(Constants.kPneumaticsType, OpConstants.kGrabberNorth1ID);
		_grabberNorth2 = new Solenoid(Constants.kPneumaticsType, OpConstants.kGrabberNorth2ID);
		_grabberSouth1 = new Solenoid(Constants.kPneumaticsType, OpConstants.kGrabberSouth1ID);
		_grabberSouth2 = new Solenoid(Constants.kPneumaticsType, OpConstants.kGrabberSouth2ID);

		if(_leftExtender == null || _rightExtender == null){
			m_Enabled = false;
			System.err.println("CLIMB FATAL: UNABLE TO INITIALIZE ONE OR BOTH EXTENDER SOLENOIDS!!!");
			System.err.println("ClimbSubsystem will be DISABLED for mechanical safety until robot code restarts!");
		}

		_swingerMasterMotor = new CANSparkMax(OpConstants.kLeftSwingerMotorID, MotorType.kBrushless);
		_swingerSlaveMotor = new CANSparkMax(OpConstants.kRightSwingerMotorID, MotorType.kBrushless);

		_northSensor = new IRSensor(OpConstants.kNorthSensorID);
		_southSensor = new IRSensor(OpConstants.kSouthSensorID);

		_swingerSlaveMotor.follow(_swingerMasterMotor);

		/**
		 * Configuring Defaults for Master/Slave motors
		 */
		_swingerMasterMotor.restoreFactoryDefaults();

		_pidMasterController = _swingerMasterMotor.getPIDController();
		_encoderMaster = _swingerMasterMotor.getEncoder();

		// set PID coefficients
		_pidMasterController.setP(CanSparkMaxConstants.kP);
		_pidMasterController.setI(CanSparkMaxConstants.kI);
		_pidMasterController.setD(CanSparkMaxConstants.kD);
		_pidMasterController.setIZone(CanSparkMaxConstants.kIz);
		_pidMasterController.setFF(CanSparkMaxConstants.kFF);
		_pidMasterController.setOutputRange(CanSparkMaxConstants.kMinOutput, CanSparkMaxConstants.kMaxOutput);

    	_pidMasterController.setSmartMotionMaxVelocity(CanSparkMaxConstants.maxVel, CanSparkMaxConstants.smartMotionSlot);
    	_pidMasterController.setSmartMotionMinOutputVelocity(CanSparkMaxConstants.minVel, CanSparkMaxConstants.smartMotionSlot);
    	_pidMasterController.setSmartMotionMaxAccel(CanSparkMaxConstants.maxAcc, CanSparkMaxConstants.smartMotionSlot);
    	_pidMasterController.setSmartMotionAllowedClosedLoopError(CanSparkMaxConstants.allowedErr, CanSparkMaxConstants.smartMotionSlot);

		//Slave defaults
		_swingerSlaveMotor.restoreFactoryDefaults();

		_pidSlaveController = _swingerSlaveMotor.getPIDController();
		_encoderSlave = _swingerSlaveMotor.getEncoder();

		// set PID coefficients
		_pidSlaveController.setP(CanSparkMaxConstants.kP);
		_pidSlaveController.setI(CanSparkMaxConstants.kI);
		_pidSlaveController.setD(CanSparkMaxConstants.kD);
		_pidSlaveController.setIZone(CanSparkMaxConstants.kIz);
		_pidSlaveController.setFF(CanSparkMaxConstants.kFF);
		_pidSlaveController.setOutputRange(CanSparkMaxConstants.kMinOutput, CanSparkMaxConstants.kMaxOutput);

    	_pidSlaveController.setSmartMotionMaxVelocity(CanSparkMaxConstants.maxVel, CanSparkMaxConstants.smartMotionSlot);
    	_pidSlaveController.setSmartMotionMinOutputVelocity(CanSparkMaxConstants.minVel, CanSparkMaxConstants.smartMotionSlot);
    	_pidSlaveController.setSmartMotionMaxAccel(CanSparkMaxConstants.maxAcc, CanSparkMaxConstants.smartMotionSlot);
    	_pidSlaveController.setSmartMotionAllowedClosedLoopError(CanSparkMaxConstants.allowedErr, CanSparkMaxConstants.smartMotionSlot);

		updateSmartDashboard();
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
		_leftExtender.set(up);
		_rightExtender.set(up);
	}

	private void setNorthGrabber(GrabberHalf grabberHalf, boolean closed){
		if(grabberHalf.value >= 1){
			_grabberNorth1.set(closed);
		}
		if(grabberHalf.value >= 2){
			_grabberNorth2.set(closed);
		}
	}

	private void setNorthGrabbers(boolean closed){
		setNorthGrabber(GrabberHalf.BOTH, closed);
	}

	private void setSouthGrabber(GrabberHalf grabberHalf, boolean closed){
		if(grabberHalf.value >= 1){
			_grabberSouth1.set(closed);
		}
		if(grabberHalf.value >= 2){
			_grabberSouth2.set(closed);
		}
	}

	private void setSouthGrabbers(boolean closed){
		setSouthGrabber(GrabberHalf.BOTH, closed);
	}

	private void startSwing(){
		_pidMasterController.setReference(
			CanSparkMaxConstants.kFwdSteps * _inputDirection.value,
			CANSparkMax.ControlType.kSmartMotion
		);
	}

	private void stopSwing(){
		_swingerMasterMotor.set(0);
		_encoderMaster.setPosition(0); // not sure if this is needed
	}

	//#endregion

	//#region State Handlers

	private boolean handleReady(){
		// Extenders down, north grabbers open, south grabbers closed
		setExtenders(false);
		setNorthGrabbers(false);
		setSouthGrabbers(false);
		stopSwing();
		return _inputDirection == InputDirection.UP;
	}

	private boolean handleExtend(){
		// Extenders up, north grabber one half closed, south grabbers closed
		setExtenders(true);
		if(_inputDirection == InputDirection.UP){
			setNorthGrabber(GrabberHalf.EAST, true);
			setSouthGrabbers(true);
			//TODO: Test the sensor
			return _sensorOverride || (_northSensor != null && _northSensor.isTriggered());
		} else {
			setNorthGrabbers(false);
			return Timer.getFPGATimestamp() - _timer >= 1;
		}
	}

	private boolean handleGrabBar(){
		// Extenders up, north grabber closed, south grabber closed
		setExtenders(true);
		if(_inputDirection == InputDirection.UP){
			setNorthGrabbers(true);
		} else if(_inputDirection == InputDirection.DOWN){
			stopSwing();
		}

		//TODO: Test this timing, we want the user to have enough time to react if the grabber doesn't grasp the bar correctly
		return Timer.getFPGATimestamp() - _timer >= 1;
	}

	private boolean handleSwingToNextBar(){
		// Extenders up, north grabber closed, south grabber half open, swinger motors spinning+
		setExtenders(true);
		startSwing();
		if(_inputDirection == InputDirection.UP){
			setSouthGrabber(GrabberHalf.EAST, true);
			//TODO: Test the timing of this
			return _sensorOverride || (_southSensor != null && _southSensor.isTriggered());
		} else {
			setNorthGrabber(GrabberHalf.EAST, true);
			return _sensorOverride || (_northSensor != null && _northSensor.isTriggered());
		}
	}

	private boolean handleGrabNextBar(){
		// Extenders up, north grabber closed, south grabber closed
		setExtenders(true);
		if(_inputDirection == InputDirection.UP){
			setSouthGrabbers(true);
			stopSwing();
		} else if(_inputDirection == InputDirection.DOWN){
			setSouthGrabbers(false);
		}
		//TODO: Test this timing
		return Timer.getFPGATimestamp() - _timer >= 1;
	}

	private boolean handleReleasePreviousBar(){
		// Extenders up, north grabber open, south grabber closed
		setExtenders(true);
		if(_inputDirection == InputDirection.UP){
			setNorthGrabbers(false);
		} else if(_inputDirection == InputDirection.DOWN){
			setNorthGrabbers(true);
			stopSwing();
		}
		//TODO: Test this timing
		return Timer.getFPGATimestamp() - _timer >= 1;
	}

	private boolean handleHanging(){
		// Extenders up, north grabber open, south grabber closed
		setExtenders(true);
		setNorthGrabbers(false);
		setSouthGrabbers(true);
		stopSwing();

		return false;
	}

	//#endregion

	private void updateSmartDashboard(){
		SmartDashboard.putString("climb_State", _currentState.name());
		SmartDashboard.putString("climb_iDir", _inputDirection.name());
		SmartDashboard.putBoolean("climb_GrbN1",  _grabberNorth1.get());
		SmartDashboard.putBoolean("climb_GrbN2", _grabberNorth2.get());
		SmartDashboard.putBoolean("climb_GrbS1", _grabberSouth1.get());
		SmartDashboard.putBoolean("climb_GrbS2", _grabberSouth2.get());
		SmartDashboard.putBoolean("climb_ExtR", _rightExtender.get());
		SmartDashboard.putBoolean("climb_ExtL", _leftExtender.get());
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
			stopSwing();
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
			case GRAB_BAR:
				transition = handleGrabBar();
				break;

			//Loop these to climb all the way up to the top
			case SWING_TO_NEXT_BAR:
				transition = handleSwingToNextBar();
				break;
			case GRAB_NEXT_BAR:
				transition = handleGrabNextBar();
				break;
			case RELEASE_PREVIOUS_BAR:
				transition = handleReleasePreviousBar();
				break;

			case HANGING:
				transition = handleHanging();
				break;
		}

		if(transition){
			_timer = Timer.getFPGATimestamp();
			if(_currentState == State.RELEASE_PREVIOUS_BAR){
				if(_climbCount == 1){
					_currentState = State.HANGING;
				} else {
					_currentState = State.SWING_TO_NEXT_BAR;
					_climbCount += _inputDirection.value;
				}
			} else {
				_currentState = _currentState.next();
			}
			_sensorOverride = false;
			// _currentState = _inputDirection == InputDirection.UP ? _currentState.next()
			// 	: _currentState.previous();
		}
	}

	
}
