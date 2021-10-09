/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.kTICKS;


public class SwerveModule {
  public static final double kMaxAngularSpeed = Math.PI;
  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;
  public CANEncoder m_driveEncoder;
  public CANEncoder m_turningEncoder;
  private CANPIDController m_drivePIDController;
  private CANPIDController m_turningPIDController;

  //private double offsetFromAbsoluteEncoder;

  private int id;
  private Boolean isInverted = Boolean.FALSE;
  private DebugValues debugValues;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
    id = driveMotorChannel;
    debugValues = new DebugValues(id);

    if(RobotBase.isReal()){
      int smartMotionSlot = 0;
      m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
      m_driveMotor.restoreFactoryDefaults();

      /*
        Sets the current limit in Amps. The motor controller will reduce the controller voltage output
        to avoid surpassing this limit. This limit is enabled by default and used for brushless only.
        This limit is highly recommended when using the NEO brushless motor. The NEO Brushless Motor has
        a low internal resistance, which can mean large current spikes that could be enough to cause damage
        to the motor and controller. This current limit provides a smarter strategy to deal with high current
        draws and keep the motor and controller operating in a safe region. The controller can also limit the
        current based on the RPM of the motor in a linear fashion to help with controllability in closed loop
        control. For a response that is linear the entire RPM range leave limit RPM at 0.

        Parameters:
          stallLimit    The current limit in Amps at 0 RPM.
          freeLimit     The current limit at free speed (5700RPM for NEO).
          limitRPM      RPM less than this value will be set to the stallLimit,
                        RPM values greater than limitRPM will scale linearly to freeLimit
      */
     // m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
     // m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 5);
     // m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
               
      m_driveMotor.setSmartCurrentLimit(40, 40);
      m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 5);  // Velocity is in Kstatus1, 
      m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
      //m_driveMotor.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
      m_drivePIDController = m_driveMotor.getPIDController();
      m_driveEncoder = m_driveMotor.getEncoder();
      m_drivePIDController.setP(1.5e-4); //500 rpm error with 5e-5
      m_drivePIDController.setI(0);
      m_drivePIDController.setD(0);
      m_drivePIDController.setFF(0.0002); //156);
      m_drivePIDController.setOutputRange(-1, 1);
      m_drivePIDController.setSmartMotionMaxVelocity(4000, smartMotionSlot); //RPM
      m_drivePIDController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
      m_drivePIDController.setSmartMotionMaxAccel(3 * (39.37*60*5.5)/Math.PI*3, smartMotionSlot); //RPM per second first number is meters/sec2
      m_drivePIDController.setSmartMotionAllowedClosedLoopError(50, smartMotionSlot);
      

  
      m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
      m_turningMotor.restoreFactoryDefaults();
      m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5); //kstatus2 has the position
      m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);

      m_turningMotor.setSmartCurrentLimit(40, 40);

      m_turningPIDController = m_turningMotor.getPIDController();
      m_turningEncoder = m_turningMotor.getEncoder();

      m_turningMotor.setInverted(true);
      m_turningPIDController.setP(5e-5);
      m_turningPIDController.setI(0);
      m_turningPIDController.setD(0);
      m_turningPIDController.setIZone(0);
      m_turningPIDController.setFF(0.0002); //.000156
      m_turningPIDController.setOutputRange(-1, 1);
      m_turningPIDController.setSmartMotionMaxVelocity(5000, smartMotionSlot);
      m_turningPIDController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
      m_turningPIDController.setSmartMotionMaxAccel(50000, smartMotionSlot);
      m_turningPIDController.setSmartMotionAllowedClosedLoopError(0.1, smartMotionSlot); 
    }
    else{
      m_driveMotor = null;
      m_turningMotor = null;
    }

    //setAzimuthZero(0); //RDB 10FEB I don't think we want this any more -- abs encoders now
  }

public SwerveModule() {
  System.err.println("DUMMY SWERVE MODULE HAS BEEN INSTANTIATED");
}

public double getDriveEncoderPosition(){
  return m_driveEncoder.getPosition();
}

  /**
   * Set the azimuthTalon encoder relative to wheel zero alignment position. For example, if current
   * absolute encoder = 0 and zero setpoint = 2767, then current relative setpoint = -2767.
   *
   * <pre>
   *
   * relative:  -2767                               0
   *           ---|---------------------------------|-------
   * absolute:    0                               2767
   *
   * </pre>
   *
   * @param zero zero setpoint, absolute encoder position (in ticks) where wheel is zeroed.
   */
  /*
  private void setAzimuthZero(double zeroSetpointAbsoluteEncoderVoltage) { // 0.0 to 3.26, 180=1.63V
    offsetFromAbsoluteEncoder = zeroSetpointAbsoluteEncoderVoltage * 16/3.26;
    //SmartDashboard.putNumber("offsetFromAbsoluteEncoder"+id, offsetFromAbsoluteEncoder);
    //logger.info("<b>Wheel</b>: setAzimuthZero starting");
    //double azimuthSetpoint = getAzimuthAbsolutePosition() - zero;
    //ErrorCode err = azimuthTalon.setSelectedSensorPosition(azimuthSetpoint, 0, 10);
    //Errors.check(err, logger);
    //azimuthTalon.set(MotionMagic, azimuthSetpoint);
    //azimuthSpark.set(azimuthSetpoint);
    //m_pidController.setReference(azimuthSetpoint, ControlType.kSmartMotion);
    //m_turningEncoder.setPosition(0);
    
    //logger.info("<b>Wheel</b>: setAzimuthZero finished");
  }
  */

  /*public double getAzimuthAbsolutePosition() {
    //return azimuthTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    //return (int)azimuthSpark.get() & 0xFFF;
    
    double rawEncoder = 0;
    if(RobotBase.isReal()){
      rawEncoder = m_turningEncoder.getPosition();
    }
    double correctedEncoder = rawEncoder - offsetFromAbsoluteEncoder;
    return correctedEncoder;
  }
  &/

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    //return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
    double velocity = 0;
    double azimuth = 0;
    if(RobotBase.isReal()){ // RPM/60 is RPS *PI*D is inches/s * 39.37 is meter/s but it's 5.5 ticks/rev
       velocity = (m_driveEncoder.getVelocity() * Math.PI * 3.0) / (39.37 * 60.0 * 5.5);
       azimuth = -m_turningEncoder.getPosition();
    }
    double azimuthPercent = Math.IEEEremainder(azimuth, kTICKS)/kTICKS;

    if(RobotBase.isReal()){
      //SmartDashboard.putNumber("Module"+id+" Drive Encoder Tick", m_driveEncoder.getPosition());
    }

    return new SwerveModuleState(velocity, new Rotation2d(azimuthPercent * 2.0 * Math.PI));

  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    //Assuming this commented block was example code and we wrote our own - SCH2021
    // Calculate the drive output from the drive PID controller.
    //final var driveOutput = m_drivePIDController.calculate(
    //    m_driveEncoder.getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    //final var turnOutput = m_turningPIDController.calculate(
    //    m_turningEncoder.get(), state.angle.getRadians()
    //);

    // Calculate the turning motor output from the turning PID controller.
    //m_driveMotor.set(driveOutput);
    //m_turningMotor.set(turnOutput);

    double azimuth = -state.angle.getDegrees() * kTICKS/360.0;
    double speedMetersPerSecond = state.speedMetersPerSecond;
    //SmartDashboard.putNumber("SpeedMPS-"+id, speedMetersPerSecond);
    // meters per sec * 39.37 is inches/s * 60 is inches per min / PI*D is RPM * 5.5 is ticks
    double drive = (speedMetersPerSecond * 5.5 * 39.37  * 60.0) / (3.0 * Math.PI);
    //wheel.set(-angleDegrees/360, speedMetersPerSecond * 16.0 * 39.37  * 60.0 / 3.0 / Math.PI);
    double azimuthPosition = 0;
    if(RobotBase.isReal()){
      azimuthPosition = m_turningEncoder.getPosition();
    }
    double azimuthError = Math.IEEEremainder(azimuth - azimuthPosition, kTICKS);

    // ********************************************************
    // minimize azimuth rotation, reversing drive if necessary
    // ********************************************************
    // synchronized(isInverted){
       isInverted = Math.abs(azimuthError) > 0.25 * kTICKS;
       if (isInverted) {
         azimuthError -= Math.copySign(0.5 * kTICKS, azimuthError);
         drive = -drive;
       }
    // }


    if(RobotBase.isReal()){
      double turningMotorOutput = azimuthPosition + azimuthError;
      m_turningPIDController.setReference(turningMotorOutput, ControlType.kSmartMotion);
      m_drivePIDController.setReference(drive, ControlType.kSmartVelocity);
      if(System.currentTimeMillis() % 100 == 0){
        SmartDashboard.putNumber("turningMotorOutput-" + id,  turningMotorOutput);
        SmartDashboard.putNumber("driveVelocityOutput-" + id,  drive);
      }

      debugValues.update(drive, turningMotorOutput, m_turningMotor.getAppliedOutput(), m_turningEncoder.getVelocity(), 
      m_driveMotor.getAppliedOutput(), m_driveEncoder.getVelocity());
    }

    //SmartDashboard.putNumber("RelativeEncoder"+id, m_turningEncoder.getPosition());
    //SmartDashboard.putNumber("absOffset"+id, offsetFromAbsoluteEncoder);
  }

  /**
   * Zeros all the SwerveModule encoders.
   */
  public void resetEncoders(double absoluteEncoderVoltage) {
 //   synchronized(isInverted){
      if(RobotBase.isReal() ){
        m_driveEncoder.setPosition(0);
        m_turningEncoder.setPosition(absoluteEncoderVoltage * 16/3.26);
      }
    }
 // }

  public DebugValues getDebugValues(){
    return debugValues;
  }

  public class DebugValues {
    public int id;
    
    public double drive;
    public double turningMotorOutput;
    public double turnAppliedOutput;
    public double turnVelocity;
    public double driveAppliedOutput;
    public double driveVelocity;

    public DebugValues(int id){
      this.id = id;
    }

    public void update(double drive, double turningMotorOutput, double turnAppliedOutput, double turnVelocity, double driveAppliedOutput, double driveVelocity) {
      this.drive = drive;
      this.turningMotorOutput = turningMotorOutput;
      this.turnAppliedOutput = turnAppliedOutput;
      this.turnVelocity = turnVelocity;
      this.driveAppliedOutput = driveAppliedOutput;
      this.driveVelocity = driveVelocity;
    }
  }

}
