package frc.robot.subsystems;

public class ClimbSubsystem extends ToggleableSubsystem {

	@Override
	protected boolean getEnabled(){
		return true;
	}

	public enum State {
		READY {
			@Override
			public State nextState(){
				return this;
			}
		};

		public abstract State nextState();
	}

	public enum InputState {
		UP,
		NEUTRAL,
		DOWN
	}

	private State _currentState = State.READY;
	private InputState _currentInputState = InputState.NEUTRAL;

	public ClimbSubsystem() {
		if(isDisabled()){
			return;
		}

	}

	public State getState(){
		if(isDisabled()){
			return State.READY;
		}

		return _currentState;
	}

	public InputState getInputState(){
		if(isDisabled()){
			return InputState.NEUTRAL;
		}

		return _currentInputState;
	}

	public void setInputState(InputState value){
		if(isDisabled()){
			return;
		}
		
		_currentInputState = value;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if(isDisabled()){
			return;
		}

		switch(_currentState){
			case READY:
				//TODO: Perform check to see if we can go into next state
				_currentState = _currentState.nextState();
				break;
		}
	}

	
}
