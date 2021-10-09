package frc.robot.subsystems;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class DummySwerveModule extends SwerveModule {
  public static final double kMaxAngularSpeed = Math.PI;
  public CANEncoder m_driveEncoder;
  public CANEncoder m_turningEncoder;

  private double offsetFromAbsoluteEncoder;

  //private int id;
  SwerveModuleState dummyState = new SwerveModuleState(0, new Rotation2d(0));

  public DummySwerveModule(int driveMotorChannel, int turningMotorChannel) {
    super();
    //id = driveMotorChannel;
  }

  public double getDriveEncoderPosition(){
    return 0;
  }

  private void setAzimuthZero(double zeroSetpointAbsoluteEncoderVoltage) { // 0.0 to 3.26, 180=1.63V
    offsetFromAbsoluteEncoder = zeroSetpointAbsoluteEncoderVoltage * 16/3.26;
  }

  public double getAzimuthAbsolutePosition() {
    double rawEncoder = 0;
    if(RobotBase.isReal()){
      rawEncoder = m_turningEncoder.getPosition();
    }
    double correctedEncoder = rawEncoder - offsetFromAbsoluteEncoder;
    return correctedEncoder;
  }

  public SwerveModuleState getState() {
    return dummyState;
  }

  public void setDesiredState(SwerveModuleState state) {
    dummyState = state;
  }

  public void resetEncoders(double absoluteEncoderVoltage) {
    setAzimuthZero(absoluteEncoderVoltage); //remember our offset
  }

}
