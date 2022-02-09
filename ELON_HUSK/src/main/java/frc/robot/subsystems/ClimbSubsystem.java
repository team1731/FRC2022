package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import com.ctre.phoenix.motorcontrol.*;


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

		//Change the range to Master/Slave motor

		_extender1 = new Solenoid(Constants.kPneumaticsType, 0);
		_extender2 = new Solenoid(Constants.kPneumaticsType, 0);
		_grabber1 = new Solenoid(Constants.kPneumaticsType, 0);
		_grabber2 = new Solenoid(Constants.kPneumaticsType, 0);

		_swingerMasterMotor = new TalonFX(0);
		_swingerSlaveMotor = new TalonFX(0);

		_swingerSlaveMotor.follow(_swingerMasterMotor);

		/**
		 * Configuring Defaults for Master/Slave motors
		 */
		_swingerMasterMotor.configFactoryDefault();

		_swingerMasterMotor.configNeutralDeadband(0.001, OpConstants.kTimeoutMs);

		_swingerMasterMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 12, 1.0));
		_swingerMasterMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 2, 3, 0.5));

		_swingerMasterMotor.setNeutralMode(NeutralMode.Brake); // Netural Mode override

		_swingerMasterMotor.configSelectedFeedbackSensor(
			TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type 
			OpConstants.kPIDLoopIdx,      			// PID Index
			OpConstants.kTimeoutMs);      			// Config Timeout

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_swingerMasterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, OpConstants.kTimeoutMs);
		_swingerMasterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, OpConstants.kTimeoutMs);
		
		/* Gains for Position Closed Loop servo */
		_swingerMasterMotor.config_kP(OpConstants.SLOT_0, OpConstants.kGains_Range.kP, OpConstants.kTimeoutMs);
		_swingerMasterMotor.config_kI(OpConstants.SLOT_0, OpConstants.kGains_Range.kI, OpConstants.kTimeoutMs);
		_swingerMasterMotor.config_kD(OpConstants.SLOT_0, OpConstants.kGains_Range.kD, OpConstants.kTimeoutMs);
		_swingerMasterMotor.config_kF(OpConstants.SLOT_0, OpConstants.kGains_Range.kF, OpConstants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		_swingerMasterMotor.configMotionCruiseVelocity(OpConstants.MMCruiseVelocity, OpConstants.kTimeoutMs);
		_swingerMasterMotor.configMotionAcceleration(OpConstants.MMAcceleration, OpConstants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_swingerMasterMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, OpConstants.kTimeoutMs);
		_swingerMasterMotor.configMotionSCurveStrength(OpConstants.MMScurve);

		//Slave Branch

		_swingerSlaveMotor.configFactoryDefault();

		_swingerSlaveMotor.configNeutralDeadband(0.001, OpConstants.kTimeoutMs);

		_swingerSlaveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 12, 1.0));
		_swingerSlaveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 2, 3, 0.5));

		_swingerSlaveMotor.setNeutralMode(NeutralMode.Brake); // Netural Mode override

		_swingerSlaveMotor.configSelectedFeedbackSensor(
			TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type 
			OpConstants.kPIDLoopIdx,      			// PID Index
			OpConstants.kTimeoutMs);      			// Config Timeout

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_swingerSlaveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, OpConstants.kTimeoutMs);
		
		/* Gains for Position Closed Loop servo */
		_swingerSlaveMotor.config_kP(OpConstants.SLOT_0, OpConstants.kGains_Range.kP, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.config_kI(OpConstants.SLOT_0, OpConstants.kGains_Range.kI, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.config_kD(OpConstants.SLOT_0, OpConstants.kGains_Range.kD, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.config_kF(OpConstants.SLOT_0, OpConstants.kGains_Range.kF, OpConstants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		_swingerSlaveMotor.configMotionCruiseVelocity(OpConstants.MMCruiseVelocity, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.configMotionAcceleration(OpConstants.MMAcceleration, OpConstants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_swingerSlaveMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.configMotionSCurveStrength(OpConstants.MMScurve);
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
