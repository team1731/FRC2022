/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* Intake Motor (I), Sequence Motor (S)
  Lo Mid Hi | iU/D iR Seq
   0  0  0  |   D   R  Off  ==> case A
   0  0  1  |   D   R  Off  ==> case A
   0  1  0  |   D   -  On   ==> case B
   0  1  1  |   D   R  Off  ==> case A
   1  0  0  |   D   -  On   ==> case B
   1  0  1  |   U   -  Off  ==> case C
   1  1  0  |   D   -  On   ==> case B
   1  1  1  |   U   -  Off  ==> case C

   If you switch case 2 & 3, then spacing will be ball width, now it's sensor width
*/
package frc.robot.commands;

import frc.robot.subsystems.SequencerSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Intake1ball extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SequencerSubsystem m_SeqSubsystem;
  //private boolean low;
  //private boolean mid;
  //private boolean high;

  /**
   * Creates a new Intake Sequence Command.
   *
   * @param seqSubsystem The sequencer subsystem this command will run on
   */
  public Intake1ball(SequencerSubsystem seqSubsystem) {
    m_SeqSubsystem = seqSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(seqSubsystem);
  }

  // Called when the command is initially scheduled.
  // If it is used as Default command then it gets call all the time
  @Override
  public void initialize() {
    m_SeqSubsystem.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SeqSubsystem.intakeBall();
    
    /* if low and high not tripped do something
    // if they are both tripped we do NOTHING
    if(!m_SeqSubsystem.lowSensorHasBall() && !m_SeqSubsystem.highSensorHasBall() ){
      //do something
      if (m_SeqSubsystem.lowSensorHasBall() ||  m_SeqSubsystem.midSensorHasBall()) {
        m_IntakeSubsystem.inactive();
      } else {
        m_IntakeSubsystem.active();
      }
      System.out.println("intake extended");
    }
    else if (m_SeqSubsystem.lowSensorHasBall() && m_SeqSubsystem.highSensorHasBall()){
      m_IntakeSubsystem.inactive();
      m_IntakeSubsystem.retract();
    }
    if((m_SeqSubsystem.lowSensorHasBall() || m_SeqSubsystem.midSensorHasBall()) && !m_SeqSubsystem.highSensorHasBall()){
      m_SeqSubsystem.forward(false); // false indicates forward intaking speed (i.e., NOT shooting)
    }
    else{
      m_SeqSubsystem.stop();
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("in end of intake1ball. interrupted="+interrupted);

    //System.out.println("IntakeSequenceCommand end interrupted=" + (interrupted?"true":"false"));
    m_SeqSubsystem.stop();
    m_SeqSubsystem.disableInterrupts();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("atsetpoint", m_SeqSubsystem.atSetpoint());
    SmartDashboard.putNumber("encoder",m_SeqSubsystem.getencoder());
    return (m_SeqSubsystem.atSetpoint() || m_SeqSubsystem.highSensorHasBall());
    
  }
}
