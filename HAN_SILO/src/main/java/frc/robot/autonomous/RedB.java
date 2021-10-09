package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class RedB extends GalacticConfiguration {
    
    @Override
    public Translation2d[] getBallPositions(){
        return new Translation2d[] {
            new Translation2d(0.05, -0.79),
            new Translation2d(0.94, -0.087),
            new Translation2d(-0.056, 0.095)
        };
    }

}
