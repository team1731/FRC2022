package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.IRSensor;
import frc.robot.Constants.OpConstants;

public class ClimbSubsystem extends ToggleableSubsystem {

	//#region ToggleableSubsystem
	@Override
	protected boolean getEnabled(){
		return false;
	}
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

	private double timer = System.currentTimeMillis();

	//#region Enums

	public enum State {
		READY(0),                // Extenders down, north grabbers open, south grabbers closed
		EXTEND(1),               // Extenders up, north grabber one half closed, south grabbers closed
		GRAB_BAR(2),             // Extenders up, north grabber closed, south grabber closed

		SWING_TO_NEXT_BAR(3),    // Extenders up, north grabber closed, south grabber closed, swinger motors spinning+
		OPEN_NEXT_GRABBERS(4),   // Extenders up, north grabber closed, south grabber half open, swinger motors spinning+
		GRAB_NEXT_BAR(5),        // Extenders up, north grabber closed, south grabber closed
		RELEASE_PREVIOUS_BAR(6); // Extenders up, north grabber open, south grabber closed

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
			return;
		}

		_leftExtender = new Solenoid(Constants.kPneumaticsType, OpConstants.kLeftExtenderID);
		_rightExtender = new Solenoid(Constants.kPneumaticsType, OpConstants.kRightExtenderID);
		_grabberNorth1 = new Solenoid(Constants.kPneumaticsType, OpConstants.kGrabberNorth1ID);
		_grabberNorth2 = new Solenoid(Constants.kPneumaticsType, OpConstants.kGrabberNorth2ID);
		_grabberSouth1 = new Solenoid(Constants.kPneumaticsType, OpConstants.kGrabberSouth1ID);
		_grabberSouth2 = new Solenoid(Constants.kPneumaticsType, OpConstants.kGrabberSouth2ID);

		_swingerMasterMotor = new CANSparkMax(OpConstants.kLeftSwingerMotorID, MotorType.kBrushless);
		_swingerSlaveMotor = new CANSparkMax(OpConstants.kRightSwingerMotorID, MotorType.kBrushless);

		_northSensor = new IRSensor(OpConstants.kNorthSensorID);
		_southSensor = new IRSensor(OpConstants.kSouthSensorID);

		_swingerSlaveMotor.follow(_swingerMasterMotor);
	}

	public State getState(){
		if(isDisabled()){
			return State.READY;
		}

		return _currentState;
	}

	public void setInputDirection(InputDirection value){
		if(isDisabled()){ return; }
		
		_inputDirection = value;
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
		_swingerMasterMotor.set(10 * _inputDirection.value);
	}

	private void stopSwing(){
		_swingerMasterMotor.set(0);
	}

	//#endregion

	//#region State Handlers

	private boolean handleReady(){
		// Extenders down, north grabbers open, south grabbers closed
		setExtenders(false);
		setNorthGrabbers(false);
		setSouthGrabbers(false);
		stopSwing();
		return true;
	}

	private boolean handleExtend(){
		// Extenders up, north grabber one half closed, south grabbers closed
		setExtenders(true);
		setNorthGrabber(GrabberHalf.EAST, true);
		setNorthGrabber(GrabberHalf.WEST, false);
		setSouthGrabbers(true);
		stopSwing();

		//TODO: Test the sensor
		return _northSensor.isTriggered();
	}

	private boolean handleGrabBar(){
		// Extenders up, north grabber closed, south grabber closed
		setExtenders(true);
		setNorthGrabbers(true);
		setSouthGrabbers(true);
		stopSwing();

		//TODO: Test this timing, we want the user to have enough time to react if the grabber doesn't grasp the bar correctly
		return System.currentTimeMillis() - timer >= 1;
	}

	private boolean handleSwingToNextBar(){
		// Extenders up, north grabber closed, south grabber closed, swinger motors spinning+
		setExtenders(true);
		setNorthGrabbers(true);
		setSouthGrabbers(true);
		startSwing();

		//TODO: We need to figure out if there's a switch for when the grabbers should open. Maybe count motor rotations?
		return System.currentTimeMillis() - timer >= 1.5;

	}

	private boolean handleOpenNextGrabbers(){
		// Extenders up, north grabber closed, south grabber half open, swinger motors spinning+
		setExtenders(true);
		setNorthGrabbers(true);
		setSouthGrabber(GrabberHalf.EAST, true);
		setSouthGrabber(GrabberHalf.WEST, false);
		startSwing();

		//TODO: Test the timing of this
		return _southSensor.isTriggered();
	}

	private boolean handleGrabNextBar(){
		// Extenders up, north grabber closed, south grabber closed
		setExtenders(true);
		setNorthGrabbers(true);
		setSouthGrabbers(true);
		stopSwing();

		//TODO: Test this timing
		return System.currentTimeMillis() - timer >= 0.5;
	}

	private boolean handleReleasePreviousBar(){
		// Extenders up, north grabber open, south grabber closed
		setExtenders(true);
		setNorthGrabbers(false);
		setSouthGrabbers(false);
		stopSwing();

		//TODO: Test this timing
		return System.currentTimeMillis() - timer >= 0.25;
	}

	//#endregion

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if(isDisabled()){ return; }

		if(_inputDirection == InputDirection.NEUTRAL){
			stopSwing();
			timer = System.currentTimeMillis();
			return;
		}

		/*
		READY(0),                // Extenders down, north grabbers open, south grabbers closed
		EXTEND(1),               // Extenders up, north grabber one half closed, south grabbers closed
		GRAB_BAR(2),             // Extenders up, north grabber closed, south grabber closed

		SWING_TO_NEXT_BAR(3),    // Extenders up, north grabber closed, south grabber closed, swinger motors spinning+
		OPEN_NEXT_GRABBERS(4),   // Extenders up, north grabber closed, south grabber half open, swinger motors spinning+
		GRAB_NEXT_BAR(5),        // Extenders up, north grabber closed, south grabber closed
		RELEASE_PREVIOUS_BAR(6); // Extenders up, north grabber open, south grabber closed
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
			case OPEN_NEXT_GRABBERS:
				transition = handleOpenNextGrabbers();
				break;
			case GRAB_NEXT_BAR:
				transition = handleGrabNextBar();
				break;
			case RELEASE_PREVIOUS_BAR:
				transition = handleReleasePreviousBar();
				break;
		}

		if(transition){
			timer = System.currentTimeMillis();
			_currentState = _currentState.next();
			// _currentState = _inputDirection == InputDirection.UP ? _currentState.next()
			// 	: _currentState.previous();
		}
	}

	
}
