package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

public class ClimbSubsystem extends ToggleableSubsystem {

	//#region ToggleableSubsystem
	@Override
	protected boolean getEnabled(){
		return false;
	}
	//#endregion

	private State _currentState = State.READY;
	private InputDirection _inputDirection = InputDirection.NEUTRAL;

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
		SWING_LAST(9);

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
		if(isDisabled()){ return; }
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

	//#region State Periodics

	private boolean handleReady(){
		return true;
	}

	private boolean handleExtend(){
		//Make extender motors go _inputDirection.value

		// if(doneExtending){
		// 	return true;
		// }

		return false;
	}

	private boolean handleGrabFirst(){
		//Make first grabber go _inputDirection.value

		// if(doneGrabbing){
		// 	return true;
		// }

		return false;
	}

	private boolean handleSwingFirst(){
		//Make swing motor go _inputDirection.value

		// if(doneSwinging){
		// 	return true;
		// }

		return false;
	}

	private boolean handleGrabSecond(){
		//Make second grabber go _inputDirection.value

		// if(doneGrabbing){
		// 	return true;
		// }

		return false;
	}

	private boolean handleReleaseFirst(){
		//Make first grabber go negative _inputDirection.value

		// if(doneReleasing){
		// 	return true;
		// }

		return false;
	}

	private boolean handleSwingSecond(){
		//Make swing motor go _inputDirection.value

		// if(doneSwinging){
		// 	return true;
		// }

		return false;
	}

	private boolean handleGrabLast(){
		//Make first grabber go _inputDirection.value

		// if(doneGrabbing){
		// 	return true;
		// }

		return false;
	}

	private boolean handleReleaseSecond(){
		//Make second grabber go negative _inputDirection.value

		// if(doneSwinging){
		// 	return true;
		// }

		return false;
	}

	private boolean handleSwingLast(){
		//Make swing motor go _inputDirection.value

		// if(doneSwinging){
		// 	return true;
		// }

		return false;
	}

	//#endregion

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if(isDisabled()){ return; }

		if(_inputDirection == InputDirection.NEUTRAL){
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
		}

		if(transition){
			_currentState = _inputDirection == InputDirection.UP ? _currentState.next()
				: _currentState.previous();
		}
	}

	
}
