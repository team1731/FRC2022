package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

	// private final SparkMaxPIDController _pidMasterController;
	// private final RelativeEncoder _encoderMaster;
	// private final RelativeEncoder _encoderSlave;
	// private final SparkMaxPIDController _pidSlaveController;

	private double _timer = Timer.getFPGATimestamp();
	private int _sdCount = 0;
	private boolean _sensorOverride = false;
	private boolean _rewinding = false;

	// #region Enums

	public enum State {
		READY(0), // Extenders down, north  open, south grabbers closed
		EXTEND(1), // Extenders up, north grabber one half closed, south grabbers closed
		GRAB_FIRST_BAR(2), // Extenders up, north grabber closed, south grabber closed

		SWING_TO_SECOND_BAR(3), // Extenders up, north grabber closed, south grabber closed, swinger motors
								// spinning+
		GRAB_SECOND_BAR(4), // Extenders up, north grabber closed, south grabber closed
		RELEASE_FIRST_BAR(5), // Extenders up, north grabber open, south grabber closed

		SWING_TO_THIRD_BAR(6),
		GRAB_THIRD_BAR(7),
		RELEASE_SECOND_BAR(8),

		HANG(9); // Extenders up, north grabber open, south grabber closed

		private final int _value;

		State(int value) {
			_value = value;
		}

		public State previous() {
			Optional<State> previousState = State.valueOf(this._value - 1);
			if (previousState.isPresent()) {
				return previousState.get();
			}

			return this;
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

		// UP(1),
		// NEUTRAL(0),
		// DOWN(-1);

		// public final int value;

		// InputDirection(int value){
		// this.value = value;
		// }
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
			// _pidMasterController = null;
			// _encoderMaster = null;
			// _encoderSlave = null;
			// _pidSlaveController = null;
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
		//_swingerMasterMotor = new CANSparkMax(OpConstants.kLeftSwingerMotorID, MotorType.kBrushless);
		//_swingerSlaveMotor = new CANSparkMax(OpConstants.kRightSwingerMotorID, MotorType.kBrushless);

		_swingerMasterMotor = new WPI_TalonFX(OpConstants.kLeftSwingerMotorID, Constants.kCAN_BUS_CANIVORE);
		_swingerSlaveMotor = new WPI_TalonFX(OpConstants.kRightSwingerMotorID, Constants.kCAN_BUS_CANIVORE);

		_swingerMasterMotor.configFactoryDefault();
		_swingerSlaveMotor.configFactoryDefault();

		_swingerMasterMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));
//		_swingerMasterMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,25, 30, 1.0));
		_swingerSlaveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));
//		_swingerSlaveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,25, 30, 1.0));

		/* setup a basic closed loop */
	//	_swingerMasterMotor.setNeutralMode(NeutralMode.Brake); // Netural Mode override
	//	_swingerSlaveMotor.setNeutralMode(NeutralMode.Brake); // Netural Mode override

		// Current limits
		// _swingerMasterMotor.setSmartCurrentLimit(40, 40, 0);
		// _swingerSlaveMotor.setSmartCurrentLimit(40,40, 0);
		// _swingerMasterMotor.setSecondaryCurrentLimit(40);
		// _swingerSlaveMotor.setSecondaryCurrentLimit(40);

	


	
	


		// Initiliaze sensors
		_northSensor = new IRSensor(OpConstants.kNorthSensorID);
		_southSensor = new IRSensor(OpConstants.kSouthSensorID);

		// Cylinder sensors
		_northBackCylinderSensor = new DigitalInput(1);
		_northFrontCylinderSensor = new DigitalInput(3);
		_southBackCylinderSensor = new DigitalInput(0);
		_northBackCylinderReleaseSensor = new DigitalInput(2);

		// Configuring Defaults for Master/Slave motors
//		_swingerMasterMotor.restoreFactoryDefaults();

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

		// Initialize PID controllers
	//	_pidMasterController = _swingerMasterMotor.getPIDController();
	//	_encoderMaster = _swingerMasterMotor.getEncoder();

		// set PID coefficients
		// _pidMasterController.setP(ClimbConstants.kP);
		// _pidMasterController.setI(ClimbConstants.kI);
		// _pidMasterController.setD(ClimbConstants.kD);
		// _pidMasterController.setIZone(ClimbConstants.kIz);
		// _pidMasterController.setFF(ClimbConstants.kFF);
		// _pidMasterController.setOutputRange(ClimbConstants.kMinOutput, ClimbConstants.kMaxOutput);

		// _pidMasterController.setSmartMotionMaxVelocity(ClimbConstants.maxVel, ClimbConstants.smartMotionSlot);
		// _pidMasterController.setSmartMotionMinOutputVelocity(ClimbConstants.minVel, ClimbConstants.smartMotionSlot);
		// _pidMasterController.setSmartMotionMaxAccel(ClimbConstants.maxAcc, ClimbConstants.smartMotionSlot);
		// _pidMasterController.setSmartMotionAllowedClosedLoopError(ClimbConstants.allowedErr,
		// 		ClimbConstants.smartMotionSlot);

		// Slave defaults
		// _swingerSlaveMotor.restoreFactoryDefaults();

		// _pidSlaveController = _swingerSlaveMotor.getPIDController();
		// _encoderSlave = _swingerSlaveMotor.getEncoder();

		// // set PID coefficients
		// _pidSlaveController.setP(ClimbConstants.kP);
		// _pidSlaveController.setI(ClimbConstants.kI);
		// _pidSlaveController.setD(ClimbConstants.kD);
		// _pidSlaveController.setIZone(ClimbConstants.kIz);
		// _pidSlaveController.setFF(ClimbConstants.kFF);
		// _pidSlaveController.setOutputRange(ClimbConstants.kMinOutput, ClimbConstants.kMaxOutput);

		// _pidSlaveController.setSmartMotionMaxVelocity(ClimbConstants.maxVel, ClimbConstants.smartMotionSlot);
		// _pidSlaveController.setSmartMotionMinOutputVelocity(ClimbConstants.minVel, ClimbConstants.smartMotionSlot);
		// _pidSlaveController.setSmartMotionMaxAccel(ClimbConstants.maxAcc, ClimbConstants.smartMotionSlot);
		// _pidSlaveController.setSmartMotionAllowedClosedLoopError(ClimbConstants.allowedErr,
		// 		ClimbConstants.smartMotionSlot);

		// _swingerSlaveMotor.setInverted(true);
		_swingerSlaveMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, 0);
		_swingerMasterMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, 0);
		// _encoderMaster.setPosition(0);
		// _encoderSlave.setPosition(0);

		setExtenders(false);
		setNorthGrabbers(false);
		setSouthGrabbers(true);
		// setNorthGrabbers(false);
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

	private void setSwingPosition(double steps){
		// _pidMasterController.setReference(steps, CANSparkMax.ControlType.kPosition);  //was kSmartmotion
		// _pidSlaveController.setReference(-steps, CANSparkMax.ControlType.kPosition);  //was kSmartMotion
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
		// Extenders down, north grabbers open, south grabbers closed
		if (_inputDirection == InputDirection.DOWN) {
			setExtenders(false);
			setNorthGrabbers(false);
			setSouthGrabbers(true);
			stopSwing();
		}
		if (_inputDirection == InputDirection.UP) {
			transition();
		}
	}

	private void handleExtend() {
		// Extenders up, north grabber one half closed, south grabbers closed
		if (_inputDirection == InputDirection.UP) {
			setExtenders(true);
			setNorthGrabber(GrabberHalf.FRONT, true);
			setNorthGrabber(GrabberHalf.BACK, false);
			setSouthGrabbers(true);
			setSwingPosition(ClimbConstants.kStartSteps);
		} else if (_inputDirection == InputDirection.DOWN) {
			setExtenders(false);
			setNorthGrabbers(false);
			setSouthGrabbers(true);
			setSwingPosition(0.0);
		} else {
			stopSwing();
		}

		if (_sensorOverride || (_northSensor != null) && _northSensor.isTriggered()) {
			transition();
		}
	}

	private void handleGrabFirstBar() {
		// Extenders up, north grabber closed, south grabber closed
		if (_inputDirection == InputDirection.UP) {
			setExtenders(true);
			setNorthGrabbers(true);
			if (Timer.getFPGATimestamp() - _timer >= 0.5 && !_northBackCylinderSensor.get()) {
				transition();
			}
		} else if (_inputDirection == InputDirection.DOWN) {
			setNorthGrabbers(false);
			transition(State.EXTEND);
		}

	}

	private void handleSwingToSecondBar() {
		// Extenders up, north grabber closed, south grabber half open, swinger motors
		// spinning+
		if (_inputDirection == InputDirection.UP) {
			setExtenders(true);
			setSwingPosition(ClimbConstants.kSecondBarSteps);
			setSouthGrabber(GrabberHalf.BACK, false);
			if (_sensorOverride || (_southSensor != null && _southSensor.isTriggered())) {
				transition();
			}
		} else if (_inputDirection == InputDirection.DOWN) {
			setSwingPosition(ClimbConstants.kStartSteps);
			if (Timer.getFPGATimestamp() - _timer >= 5) {
				transition(State.EXTEND);
			}
		}
	}

	private void handleGrabSecondBar() {
		// Extenders up, north grabber closed, south grabber closed
		setExtenders(true);
		setSouthGrabbers(true);
		if (Timer.getFPGATimestamp() - _timer >= 2) {
			stopSwing();
		}

		if (Timer.getFPGATimestamp() - _timer >= .5 && !_southBackCylinderSensor.get()) {
			transition();
		}

	}

	private void handleReleaseFirstBar() {
		// Extenders up, north grabber open, south grabber closed
		if (_inputDirection == InputDirection.UP) {
			setExtenders(true);
			setNorthGrabbers(false);
			setSwingPosition(ClimbConstants.kSecondBarSteps - ClimbConstants.kBckSteps);
		} else {
			stopSwing();
		}

		if (!_northBackCylinderReleaseSensor.get()) {
			transition();
		}
	}

	private void handleSwingToThirdBar() {
		if (_inputDirection == InputDirection.UP) {
			setExtenders(true);
			setNorthGrabber(GrabberHalf.FRONT, false);
			if (Timer.getFPGATimestamp() - _timer >= 2) {
				setNorthGrabber(GrabberHalf.BACK, true);
				if (_sensorOverride || (_northSensor != null && _northSensor.isTriggered())) {
					transition();
				}
			}
			if ((Timer.getFPGATimestamp() - _timer >= 3.0) &&  (_northBackCylinderSensor.get())) {
				stopSwing();
			} else {
				setSwingPosition(ClimbConstants.kThirdBarSteps);
			}
		} else {
			stopSwing();
		}
	}

	private void handleGrabThirdBar() {
		setExtenders(true);
		setNorthGrabbers(true);
		stopSwing();

		if (Timer.getFPGATimestamp() - _timer >= 0.5 && !_northFrontCylinderSensor.get()) {
			transition();
		}
	}

	private void handleReleaseSecondBar() {

		if (_inputDirection == InputDirection.UP) {
			setExtenders(true);
			setSouthGrabbers(false);
			setSwingPosition(ClimbConstants.kThirdBarSteps - ClimbConstants.kBckSteps);

		} else {
			stopSwing();
		}
		
		if (Timer.getFPGATimestamp() - _timer >= 2) {
			transition();
		}
	}

	private void handleHang() {
		// Extenders up, north grabber open, south grabber closed
		setExtenders(true);
		setNorthGrabbers(true);
		setSouthGrabbers(false);

		if (_inputDirection == InputDirection.UP) {
			setSwingPosition(ClimbConstants.kHangSteps);
		} else {
			stopSwing();
		}
	}

	// #endregion

	public void doSD() {
		if (_sdCount++ > 50) {
		SmartDashboard.putNumber("climb_North Sensor Voltage", _northSensor != null ? _northSensor.getVoltage() : 0);
		SmartDashboard.putBoolean("climb_North Sensor Triggered",
				_northSensor != null ? _northSensor.isTriggered() : false);
		SmartDashboard.putNumber("climb_South Sensor Voltage", _southSensor != null ? _southSensor.getVoltage() : 0);
		SmartDashboard.putBoolean("climb_South Sensor Triggered",
				_southSensor != null ? _southSensor.isTriggered() : false);
		SmartDashboard.putBoolean("climb_Sensor Override", _sensorOverride);
		SmartDashboard.putBoolean("climb_South_Back_Cylinder", _southBackCylinderSensor.get());
		SmartDashboard.putBoolean("climb_North_Back_Cylinder", _northBackCylinderSensor.get());
		SmartDashboard.putBoolean("climb_North_Front_Cylinder", _northFrontCylinderSensor.get());
		SmartDashboard.putBoolean("climb_North_Back_Release_Cylinder", _northBackCylinderReleaseSensor.get());
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
		// _encoderMaster.setPosition(0);
		// _encoderSlave.setPosition(0);
	}

}
