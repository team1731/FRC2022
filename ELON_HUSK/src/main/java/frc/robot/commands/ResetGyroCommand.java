package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ResetGyroCommand extends InstantCommand {
    
    private final DriveSubsystem _drive;

    public ResetGyroCommand(DriveSubsystem drive){
        _drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        _drive.resetGyro();
        System.out.println("Gyro reset by user");
    }

}
