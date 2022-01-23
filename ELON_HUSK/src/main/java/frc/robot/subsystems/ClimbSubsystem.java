package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class ClimbSubsystem extends ToggleableSubsystem {

	//#region ToggleableSubsystem
	@Override
	protected boolean getEnabled(){
		return false;
	}
	//#endregion

	private State _currentState = State.READY;
	private InputDirection _inputDirection = InputDirection.NEUTRAL;

	private final Solenoid _extender1;
	private final Solenoid _extender2;
	private final Solenoid _grabber1;
	private final Solenoid _grabber2;

	private final TalonFX _swingerMasterMotor;
	private final TalonFX _swingerSlaveMotor;

	//#region Enums

	public enum State {
		READY(0),
		EXTEND(1),
		GRAB_FIRST(2),
		SWING_FIRST(3),
		GRAB_SECOND(4),
		RELEASE_FIRST(5),
		SWING_SECOND(6),
		GRAB_LAST(7),
		RELEASE_SECOND(8),
		SWING_LAST(9),
		FINISHED(10);

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

	//#endregion

	public ClimbSubsystem() {
		if(isDisabled()){ 
			_extender1 = null;
			_extender2 = null;
			_grabber1 = null;
			_grabber2 = null;
			_swingerMasterMotor = null;
			_swingerSlaveMotor = null;
			return;
		}

		_extender1 = new Solenoid(Constants.kPneumaticsType, 0);
		_extender2 = new Solenoid(Constants.kPneumaticsType, 0);
		_grabber1 = new Solenoid(Constants.kPneumaticsType, 0);
		_grabber2 = new Solenoid(Constants.kPneumaticsType, 0);

		_swingerMasterMotor = new TalonFX(0);
		_swingerSlaveMotor = new TalonFX(0);

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

	private void setExtenders(boolean on){
		_extender1.set(on);
		_extender2.set(on);
	}

	private void grab(int id){
		if(id == 1){
			_grabber1.set(_inputDirection == InputDirection.UP);
		} else {
			_grabber2.set(_inputDirection == InputDirection.UP);
		}
	}

	private void release(int id){
		if(_inputDirection == InputDirection.DOWN){
			grab(id);
			return;
		}

		if(id == 1){
			_grabber1.set(_inputDirection == InputDirection.DOWN);
		} else {
			_grabber2.set(_inputDirection == InputDirection.DOWN);
		}
	}
	
	private void startSwing(){
		_swingerMasterMotor.set(ControlMode.PercentOutput, 100 * _inputDirection.value);
	}

	private void stopSwing(){
		_swingerMasterMotor.set(ControlMode.PercentOutput, 0);
	}

	//#region State Handlers

	private boolean handleReady(){
		setExtenders(false);
		release(1);
		release(2);
		_swingerMasterMotor.set(ControlMode.PercentOutput, 0);
		return true;
	}

	private boolean handleExtend(){
		//Make extender motors go _inputDirection.value

		setExtenders(_inputDirection == InputDirection.UP);

		// if(doneExtending){
		// 	return true;
		// }

		return false;
	}

	private boolean handleGrabFirst(){
		//Make first grabber go _inputDirection.value

		grab(1);
		stopSwing();

		// if(doneGrabbing){
		// 	return true;
		// }

		return false;
	}

	private boolean handleSwingFirst(){
		//Make swing motor go _inputDirection.value

		startSwing();

		// if(doneSwinging){
		// 	return true;
		// }

		return false;
	}

	private boolean handleGrabSecond(){
		//Make second grabber go _inputDirection.value

		grab(2);
		stopSwing();

		// if(doneGrabbing){
		// 	return true;
		// }

		return false;
	}

	private boolean handleReleaseFirst(){
		//Make first grabber go negative _inputDirection.value

		release(1);
		stopSwing();

		// if(doneReleasing){
		// 	return true;
		// }

		return false;
	}

	private boolean handleSwingSecond(){
		//Make swing motor go _inputDirection.value

		startSwing();

		// if(doneSwinging){
		// 	return true;
		// }

		return false;
	}

	private boolean handleGrabLast(){
		//Make first grabber go _inputDirection.value

		stopSwing();
		grab(2);

		// if(doneGrabbing){
		// 	return true;
		// }

		return false;
	}

	private boolean handleReleaseSecond(){
		//Make second grabber go negative _inputDirection.value

		release(1);

		// if(doneSwinging){
		// 	return true;
		// }

		return false;
	}

	private boolean handleSwingLast(){
		//Make swing motor go _inputDirection.value

		startSwing();

		// if(doneSwinging){
		// 	return true;
		// }

		return false;
	}

	private boolean handleFinished(){
		stopSwing();
		
		return _inputDirection == InputDirection.DOWN;
	}

	//#endregion

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if(isDisabled()){ return; }

		if(_inputDirection == InputDirection.NEUTRAL){
			_swingerMasterMotor.set(ControlMode.PercentOutput, 0);
			return;
		}

		boolean transition = false;
		switch(_currentState){
			case READY:
				transition = handleReady();
				break;
			case EXTEND:
				transition = handleExtend();
				break;
			case GRAB_FIRST:
				transition = handleGrabFirst();
				break;
			case SWING_FIRST:
				transition = handleSwingFirst();
				break;
			case GRAB_SECOND:
				transition = handleGrabSecond();
				break;
			case RELEASE_FIRST:
				transition = handleReleaseFirst();
				break;
			case SWING_SECOND:
				transition = handleSwingSecond();
				break;
			case GRAB_LAST:
				transition = handleGrabLast();
				break;
			case RELEASE_SECOND:
				transition = handleReleaseSecond();
				break;
			case SWING_LAST:
				transition = handleSwingLast();
				break;
			case FINISHED:
				transition = handleFinished();
				break;
		}

		if(transition){
			_currentState = _inputDirection == InputDirection.UP ? _currentState.next()
				: _currentState.previous();
		}
	}

	
}
