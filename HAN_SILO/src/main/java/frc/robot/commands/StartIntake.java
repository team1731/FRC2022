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
public class StartIntake extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeSubsystem m_IntakeSubsystem;

	// private final SequencerSubsystem m_SeqSubsystem;
	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param intakeSubsystem The intake subsystem this command will run on
	 * @param seqSubsystem    The sequencer subsystem this command will run on
	 */
	public StartIntake(IntakeSubsystem intakeSubsystem, SequencerSubsystem seqSubsystem) {
		m_IntakeSubsystem = intakeSubsystem;
		// m_SeqSubsystem = seqSubsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		if (intakeSubsystem != null && seqSubsystem != null) {
			addRequirements(intakeSubsystem, seqSubsystem);
		}
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_IntakeSubsystem.extend();
		// m_SeqSubsystem.stop();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// m_SeqSubsystem.addPowerCell();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_IntakeSubsystem.retract();
		// m_SeqSubsystem.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return m_SeqSubsystem.getMaxPowerCells();
		return false;
	}
}
