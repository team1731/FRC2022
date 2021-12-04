/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeExtend extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeExtend(IntakeSubsystem subsystem) {
    m_intakeSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    if(subsystem != null){
      addRequirements(subsystem);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.extend();
  }

  /*
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  */
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
