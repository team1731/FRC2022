package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import frc.robot.Constants.OpConstants.LedOption;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class IntakeSubsystem extends SubsystemBase {

  //private final LedStringSubsystem m_ledstring;
  private final PWMTalonFX mTalonIntake;
  private final DoubleSolenoid mIntakeSolenoid;
  private String mTalonState;
  
  /**
   * Creates a new IntakeSubsystem.
 * @param m_ledstring
   */
	public IntakeSubsystem(/*LedStringSubsystem m_ledstring*/) {
    //this.m_ledstring = m_ledstring;
    mTalonIntake = new PWMTalonFX(OpConstants.kMotorPWMIntake);
    mIntakeSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1IntakeRetract, OpConstants.k1IntakeExtend);
    mTalonState = "Off";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Enables the intake by extending solenoid & turning on motor.
   */
  public void extend() {
    mIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    mTalonState = "Intake Extend";
  }

  public void active() {
    mTalonIntake.setSpeed(OpConstants.kMotorIntakeFwdSpeed);
    mTalonState = "Intake Fwd";
    //m_ledstring.option(LedOption.INTAKE);
  }

  public void inactive() {
    mTalonIntake.setSpeed(0);
    //mTalonIntake.stopMotor();
    mTalonState = "Intake Stop";
  }

  /**
   * Enables the intake by extending solenoid & turning on motor.
   */
  public void eject() {
    mTalonIntake.setSpeed(OpConstants.kMotorIntakeRevSpeed);
    mTalonState = "Intake Rev";
  }

  /**
   * Enables the intake by retracting solenoid & turning off motor.
   */
  public void retract() {
    mIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    mTalonIntake.setSpeed(0);
    //mTalonIntake.stopMotor();
    mTalonState = "Intake Retracted/Off";
  }

  public String getIntakeState() {
    return(mTalonState);
  }
}
