/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class SeqEjectCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_IntakeSubsystem;
  //private final SequencerSubsystem m_SeqSubsystem;
  //private Timer mTimer;
  double elapsed;

  /**
   * Run the sequencer and intake backwards in case of jammed sequencer
   *
   * @param intakeSubsystem The intake subsystem this command will run on
   * @param seqSubsystem    The sequencer subsystem this command will run on
   */
  public SeqEjectCommand(IntakeSubsystem intakeSubsystem, SequencerSubsystem seqSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    //m_SeqSubsystem = seqSubsystem;
   // mTimer = new Timer();
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(seqSubsystem,intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  // If it is used as Default command then it gets call all the time
  @Override
  public void initialize() {
    //m_SeqSubsystem.stop();
   // mTimer.start();
   // elapsed = mTimer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.eject();
   // m_SeqSubsystem.reverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("SeqResetCommand end interrupted=" + (interrupted?"true":"false"));
  //  m_SeqSubsystem.stop();
    m_IntakeSubsystem.retract();
    m_IntakeSubsystem.inactive();
    //mTimer.stop();
  }

  // Returns true when the command should end after 2 seconds.
  @Override
  public boolean isFinished() {
   // return (mTimer.get() - elapsed) > OpConstants.kSeqEjectDelay;
   return false;
  }
}
