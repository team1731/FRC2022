package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class RedB extends GalacticConfiguration {
    
    @Override
    public Translation2d[] getBallPositions(){
        return new Translation2d[] {
            new Translation2d(0.20962, -0.74840),
            new Translation2d(0.13750, -0.64167),
            new Translation2d(0.96250, -0.34583)
        };
    }

}
