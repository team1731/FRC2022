package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.IRSensor;
import frc.robot.Constants.OpConstants;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends ToggleableSubsystem {

	// #region ToggleableSubsystem
	@Override
	protected boolean getEnabled() {
		return true;
	}
	// #endregion

	private State _currentState = State.READY;
	private InputDirection _inputDirection = InputDirection.NEUTRAL;

	private final DoubleSolenoid _extender;
	private final DoubleSolenoid _grabberNorthFront;
	private final DoubleSolenoid _grabberNorthBack;
	private final DoubleSolenoid _grabberSouthFront;
	private final DoubleSolenoid _grabberSouthBack;

	private final WPI_TalonFX _swingerMasterMotor;
	private final WPI_TalonFX _swingerSlaveMotor;

	private final IRSensor _northSensor;
	private final IRSensor _southSensor;
	private final DigitalInput _northFrontCylinderSensor;
	private final DigitalInput _northBackCylinderSensor;
	private final DigitalInput _southBackCylinderSensor;
	private final DigitalInput _northBackCylinderReleaseSensor;

	private double _timer = Timer.getFPGATimestamp();
	private int _sdCount = 0;
	private boolean _sensorOverride = false;
	private boolean _rewinding = false;

	// #region Enums

	public enum State {
		READY(0), // Extenders down, north open, south grabbers closed
		EXTEND(1), // Extenders up, north grabber one half closed, south grabbers closed
		GRAB_FIRST_BAR(2), // Extenders up, north grabber closed, south grabber closed

		SWING_TO_SECOND_BAR(3), // Extenders up, north grabber closed, south grabber closed, swinger motors
								// spinning+
		GRAB_SECOND_BAR(4), // Extenders up, north grabber closed, south grabber closed, swinger motors
							// stopped
		RELEASE_FIRST_BAR(5), // Extenders up, north grabber open, south grabber closed, swing backwards

		SWING_TO_THIRD_BAR(6), // Extenders up, north grabber open, south grabber closed, swinger motors
								// spinning+
		GRAB_THIRD_BAR(7), // Extenders up, north grabber closed, south grabber closed, swinger motors
							// stopped
		RELEASE_SECOND_BAR(8), // Extenders up, north grabber closed, south grabber open, swing backwards

		HANG(9); // Extenders up, north grabber open, south grabber closed, swing down

		private final int _value;

		State(int value) {
			_value = value;
		}

		public State next() {
			Optional<State> nextState = State.valueOf(this._value + 1);
			if (nextState.isPresent()) {
				return nextState.get();
			}

			return this;
		}

		public static Optional<State> valueOf(int value) {
			return Arrays.stream(values())
					.filter(state -> state._value == value)
					.findFirst();
		}
	}

	public enum InputDirection {
		UP,
		NEUTRAL,
		DOWN,
	}

	private enum GrabberHalf {
		FRONT,
		BACK,
		BOTH;
	}

	// #endregion

	public ClimbSubsystem() {
		if (isDisabled()) {
			_extender = null;
			_grabberNorthFront = null;
			_grabberNorthBack = null;
			_grabberSouthFront = null;
			_grabberSouthBack = null;
			_swingerMasterMotor = null;
			_swingerSlaveMotor = null;
			_northSensor = null;
			_southSensor = null;
			_northBackCylinderSensor = null;
			_northFrontCylinderSensor = null;
			_northBackCylinderReleaseSensor = null;
			_southBackCylinderSensor = null;
			return;
		}

		// Initialize solenoids
		_extender = new DoubleSolenoid(OpConstants.kPneumaticsCanID1, Constants.kPneumaticsType,
				OpConstants.kExtenderUpID, OpConstants.kExtenderDownID);
		_grabberNorthFront = new DoubleSolenoid(OpConstants.kPneumaticsCanID2, Constants.kPneumaticsType,
				OpConstants.kGrabberNorthFrontCloseID, OpConstants.kGrabberNorthFrontOpenID);
		_grabberNorthBack = new DoubleSolenoid(OpConstants.kPneumaticsCanID1, Constants.kPneumaticsType,
				OpConstants.kGrabberNorthBackCloseID, OpConstants.kGrabberNorthBackOpenID);
		_grabberSouthFront = new DoubleSolenoid(OpConstants.kPneumaticsCanID1, Constants.kPneumaticsType,
				OpConstants.kGrabberSouthFrontCloseID, OpConstants.kGrabberSouthFrontOpenID);
		_grabberSouthBack = new DoubleSolenoid(OpConstants.kPneumaticsCanID2, Constants.kPneumaticsType,
				OpConstants.kGrabberSouthBackCloseID, OpConstants.kGrabberSouthBackOpenID);

		//
		if (_extender == null) {
			throw new IllegalStateException("CLIMB FATAL: Unable to init extender DoubleSolenoid");
		}

		// Initialize motors
		_swingerMasterMotor = new WPI_TalonFX(OpConstants.kLeftSwingerMotorID, Constants.kCAN_BUS_CANIVORE);
		_swingerSlaveMotor = new WPI_TalonFX(OpConstants.kRightSwingerMotorID, Constants.kCAN_BUS_CANIVORE);

		_swingerMasterMotor.configFactoryDefault();
		_swingerSlaveMotor.configFactoryDefault();

		_swingerMasterMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));
		_swingerSlaveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));

		// Initiliaze sensors
		_northSensor = new IRSensor(OpConstants.kNorthSensorID);
		_southSensor = new IRSensor(OpConstants.kSouthSensorID);

		// Cylinder sensors
		_northBackCylinderSensor = new DigitalInput(1);
		_northFrontCylinderSensor = new DigitalInput(3);
		_southBackCylinderSensor = new DigitalInput(0);
		_northBackCylinderReleaseSensor = new DigitalInput(2);

		// Configuring Defaults for Master/Slave motors
		_swingerMasterMotor.configSelectedFeedbackSensor(
				TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type
				OpConstants.kPIDLoopIdx, // PID Index
				OpConstants.kTimeoutMs); // Config Timeout

		_swingerSlaveMotor.configSelectedFeedbackSensor(
				TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type
				OpConstants.kPIDLoopIdx, // PID Index
				OpConstants.kTimeoutMs); // Config Timeout

		_swingerMasterMotor.configNeutralDeadband(0.001, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.configNeutralDeadband(0.001, OpConstants.kTimeoutMs);

		_swingerMasterMotor.setSensorPhase(true);
		_swingerMasterMotor.setInverted(false);
		_swingerSlaveMotor.setSensorPhase(true);
		_swingerSlaveMotor.setInverted(false);

		_swingerMasterMotor.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		_swingerMasterMotor.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		_swingerMasterMotor.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		_swingerMasterMotor.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);

		_swingerSlaveMotor.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);

		_swingerMasterMotor.selectProfileSlot(OpConstants.SLOT_0, OpConstants.kPIDLoopIdx);
		_swingerMasterMotor.config_kP(OpConstants.SLOT_0, ClimbConstants.kP, OpConstants.kTimeoutMs);
		_swingerMasterMotor.config_kI(OpConstants.SLOT_0, ClimbConstants.kI, OpConstants.kTimeoutMs);
		_swingerMasterMotor.config_kD(OpConstants.SLOT_0, ClimbConstants.kD, OpConstants.kTimeoutMs);
		_swingerMasterMotor.config_kF(OpConstants.SLOT_0, ClimbConstants.kFF, OpConstants.kTimeoutMs);

		_swingerSlaveMotor.selectProfileSlot(OpConstants.SLOT_0, OpConstants.kPIDLoopIdx);
		_swingerSlaveMotor.config_kP(OpConstants.SLOT_0, ClimbConstants.kP, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.config_kI(OpConstants.SLOT_0, ClimbConstants.kI, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.config_kD(OpConstants.SLOT_0, ClimbConstants.kD, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.config_kF(OpConstants.SLOT_0, ClimbConstants.kFF, OpConstants.kTimeoutMs);

		_swingerSlaveMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, 0);
		_swingerMasterMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, 0);

		setExtenders(false);
		setNorthGrabbers(false);
		setSouthGrabbers(true);

		// Test that this is fine in the constructor. Can't do it now because
		// competition
		// handleReady();
	}

	public void setInputDirection(InputDirection value) {
		if (isDisabled())
			return;

		_timer = Timer.getFPGATimestamp();
		_inputDirection = value;
	}

	public boolean getSensorOveride() {
		if (isDisabled())
			return false;

		return _sensorOverride;
	}

	public void setSensorOverride(boolean value) {
		if (isDisabled())
			return;

		_sensorOverride = value;
	}

	// #region Controls

	private void setExtenders(boolean up) {
		_extender.set(up ? Value.kForward : Value.kReverse);
	}

	private void setNorthGrabber(GrabberHalf grabberHalf, boolean closed) {
		if (grabberHalf == GrabberHalf.FRONT || grabberHalf == GrabberHalf.BOTH) {
			_grabberNorthFront.set(closed ? Value.kForward : Value.kReverse);
		}
		if (grabberHalf == GrabberHalf.BACK || grabberHalf == GrabberHalf.BOTH) {
			_grabberNorthBack.set(closed ? Value.kForward : Value.kReverse);
		}
	}

	private void setNorthGrabbers(boolean closed) {
		setNorthGrabber(GrabberHalf.BOTH, closed);
	}

	private void setSouthGrabber(GrabberHalf grabberHalf, boolean closed) {
		if (grabberHalf == GrabberHalf.FRONT || grabberHalf == GrabberHalf.BOTH) {
			_grabberSouthFront.set(closed ? Value.kForward : Value.kReverse);
		}
		if (grabberHalf == GrabberHalf.BACK || grabberHalf == GrabberHalf.BOTH) {
			_grabberSouthBack.set(closed ? Value.kForward : Value.kReverse);
		}
	}

	private void setSouthGrabbers(boolean closed) {
		setSouthGrabber(GrabberHalf.BOTH, closed);
	}

	private void setSwingPosition(double steps) {
		_swingerMasterMotor.set(TalonFXControlMode.Position, steps);
		_swingerSlaveMotor.set(TalonFXControlMode.Position, -steps);
	}

	private void stopSwing() {
		_swingerMasterMotor.set(0);
		_swingerSlaveMotor.set(0);
	}

	// #endregion

	// #region State Handlers

	public void handleReady() {
		if (_inputDirection == InputDirection.DOWN) {
			// Ensure defaults
			setExtenders(false);
			setNorthGrabbers(false);
			setSouthGrabbers(true);
			stopSwing();
		}
		if (_inputDirection == InputDirection.UP) {
			// Immediately goes to the next state
			transition();
		}
	}

	private void handleExtend() {
		if (_inputDirection == InputDirection.UP) {
			// Extend and half open north grabbers to reach for the first bar
			setExtenders(true);
			setNorthGrabber(GrabberHalf.FRONT, true);
			setNorthGrabber(GrabberHalf.BACK, false);
			setSouthGrabbers(true);
			setSwingPosition(ClimbConstants.kStartSteps);
		} else if (_inputDirection == InputDirection.DOWN) {
			// Retract
			setExtenders(false);
			setNorthGrabbers(false);
			setSouthGrabbers(true);
			setSwingPosition(0.0);
		} else {
			// TODO: This seems unnecessary? This should already happen in periodic,
			// and I'm pretty sure this never runs if neutral... I'm leaving it until we can test
			stopSwing();
		}

		if (_sensorOverride || (_northSensor != null) && _northSensor.isTriggered()) {
			// Go to next state once the north IR sensor gets tripped by the bar
			transition();
		}
	}

	private void handleGrabFirstBar() {
		if (_inputDirection == InputDirection.UP) {
			// Grab the bar
			setExtenders(true);
			setNorthGrabbers(true);
			if (Timer.getFPGATimestamp() - _timer >= 0.5 && !_northBackCylinderSensor.get()) {
				// Wait a lil bit and wait for the cylinder to open
				transition();
			}
		} else if (_inputDirection == InputDirection.DOWN) {
			// Release the bar and go back to the EXTEND state
			setNorthGrabbers(false);
			transition(State.EXTEND);
		}

	}

	private void handleSwingToSecondBar() {
		if (_inputDirection == InputDirection.UP) {
			// Half open the other grabber and swing up to the next bar
			setExtenders(true);
			setSwingPosition(ClimbConstants.kSecondBarSteps);
			setSouthGrabber(GrabberHalf.BACK, false);
			if (_sensorOverride || (_southSensor != null && _southSensor.isTriggered())) {
				// Go to next state once the south IR sensor gets tripped by the bar
				transition();
			}
		} else if (_inputDirection == InputDirection.DOWN) {
			// Go back to start and wait 5 seconds before releasing
			setSwingPosition(ClimbConstants.kStartSteps);
			if (Timer.getFPGATimestamp() - _timer >= 5) {
				transition(State.EXTEND);
			}
		}
	}

	private void handleGrabSecondBar() {
		// At this point, we can't go back. Don't worry about InputDirection.DOWN

		// Grab the second bar
		setExtenders(true);
		setSouthGrabbers(true);
		if (Timer.getFPGATimestamp() - _timer >= 2) {
			// Not sure why this is here
			stopSwing();
		}

		if (Timer.getFPGATimestamp() - _timer >= .5 && !_southBackCylinderSensor.get()) {
			// Wait a lil bit and wait for cylinder to open.
			transition();
		}

	}

	private void handleReleaseFirstBar() {
		if (_inputDirection == InputDirection.UP) {
			setExtenders(true);
			setNorthGrabbers(false);
			setSwingPosition(ClimbConstants.kSecondBarSteps - ClimbConstants.kBckSteps);
		} else {
			// TODO: Look at handleExtend(), same comment here.
			stopSwing();
		}

		if (!_northBackCylinderReleaseSensor.get()) {
			// Wait for the grabber to release before going to next state
			transition();
		}
	}

	private void handleSwingToThirdBar() {
		if (_inputDirection == InputDirection.UP) {
			// Open the other grabber
			setExtenders(true);
			setNorthGrabber(GrabberHalf.FRONT, false);
			if (Timer.getFPGATimestamp() - _timer >= 2) {
				// Half open the other grabber after it approximately goes underneath
				setNorthGrabber(GrabberHalf.BACK, true);
				if (_sensorOverride || (_northSensor != null && _northSensor.isTriggered())) {
					// Go to next state once north IR sensor gets tripped by the bar
					transition();
				}
			}
			if ((Timer.getFPGATimestamp() - _timer >= 3.0) && (_northBackCylinderSensor.get())) {
				// Stop if we've been trying to get to the next bar for 3 seconds
				// and the grabber is still open
				stopSwing();
			} else {
				// Swing up to the next bar
				setSwingPosition(ClimbConstants.kThirdBarSteps);
			}
		} else {
			// TODO: Look at handleExtend(), same comment here
			stopSwing();
		}
	}

	private void handleGrabThirdBar() {
		// Stop swing and grab
		setExtenders(true);
		setNorthGrabbers(true);
		stopSwing();

		if (Timer.getFPGATimestamp() - _timer >= 0.5 && !_northFrontCylinderSensor.get()) {
			// Go to next state once bar is grabbed
			transition();
		}
	}

	private void handleReleaseSecondBar() {
		if (_inputDirection == InputDirection.UP) {
			// Let go of last bar
			setExtenders(true);
			setSouthGrabbers(false);
			setSwingPosition(ClimbConstants.kThirdBarSteps - ClimbConstants.kBckSteps);
		} else {
			// TODO: Look at handleExtend(), same comment here
			stopSwing();
		}

		if (Timer.getFPGATimestamp() - _timer >= 2) {
			// Go to next state once second bar is released
			transition();
		}
	}

	private void handleHang() {
		// Swing so we can get away from the third bar
		setExtenders(true);
		setNorthGrabbers(true);
		setSouthGrabbers(false);

		if (_inputDirection == InputDirection.UP) {
			setSwingPosition(ClimbConstants.kHangSteps);
		} else {
			// TODO: Look at handleExtend(), same comment here
			stopSwing();
		}
	}

	// #endregion

	public void doSD() {
		if (_sdCount++ > 50) {
			SmartDashboard.putNumber("climb_North Sensor Voltage",
					_northSensor != null ? _northSensor.getVoltage() : 0);
			SmartDashboard.putBoolean("climb_North Sensor Triggered",
					_northSensor != null ? _northSensor.isTriggered() : false);
			SmartDashboard.putNumber("climb_South Sensor Voltage",
					_southSensor != null ? _southSensor.getVoltage() : 0);
			SmartDashboard.putBoolean("climb_South Sensor Triggered",
					_southSensor != null ? _southSensor.isTriggered() : false);

			// We tend to test the IR sensors more than anything. I'll leave these disabled for now
			// SmartDashboard.putBoolean("climb_Sensor Override", _sensorOverride);
			// SmartDashboard.putBoolean("climb_South_Back_Cylinder", _southBackCylinderSensor.get());
			// SmartDashboard.putBoolean("climb_North_Back_Cylinder", _northBackCylinderSensor.get());
			// SmartDashboard.putBoolean("climb_North_Front_Cylinder", _northFrontCylinderSensor.get());
			// SmartDashboard.putBoolean("climb_North_Back_Release_Cylinder", _northBackCylinderReleaseSensor.get());
			_sdCount = 0;
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (isDisabled())
			return;

		if ((_inputDirection == InputDirection.NEUTRAL) && !_rewinding) {
			_swingerMasterMotor.set(0);
			_swingerSlaveMotor.set(0);
			_timer = Timer.getFPGATimestamp();
			if (_currentState != State.READY) {
				return;
			}

		}

		switch (_currentState) {
			case READY:
				handleReady();
				break;
			case EXTEND:
				handleExtend();
				break;
			case GRAB_FIRST_BAR:
				handleGrabFirstBar();
				break;

			case SWING_TO_SECOND_BAR:
				handleSwingToSecondBar();
				break;
			case GRAB_SECOND_BAR:
				handleGrabSecondBar();
				break;
			case RELEASE_FIRST_BAR:
				handleReleaseFirstBar();
				break;

			case SWING_TO_THIRD_BAR:
				handleSwingToThirdBar();
				break;
			case GRAB_THIRD_BAR:
				handleGrabThirdBar();
				break;
			case RELEASE_SECOND_BAR:
				handleReleaseSecondBar();
				break;

			case HANG:
				handleHang();
				break;
		}
	}

	private void transition(State nextState) {
		_timer = Timer.getFPGATimestamp();
		_sensorOverride = false;
		_currentState = nextState;
	}

	private void transition() {
		transition(_currentState.next());
	}

	public void startRewind() {
		_rewinding = true;
		_swingerMasterMotor.set(-0.5);
		_swingerSlaveMotor.set(0.5);
	}

	public void stopRewind() {
		_rewinding = false;
		transition(State.READY);
		_swingerMasterMotor.set(0);
		_swingerSlaveMotor.set(0);
		_swingerSlaveMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, 0);
		_swingerMasterMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, 0);
	}

}
