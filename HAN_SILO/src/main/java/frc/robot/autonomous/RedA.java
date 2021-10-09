package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class RedA extends GalacticConfiguration {
    
    @Override
    public Translation2d[] getBallPositions(){
        return new Translation2d[] {
            new Translation2d(0.32, -0.85),
            new Translation2d(0.64, -0.104),
            new Translation2d(-0.678, 0.29)
        };
    }

}
