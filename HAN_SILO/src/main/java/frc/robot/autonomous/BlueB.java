package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class BlueB extends GalacticConfiguration {
    
    @Override
    public Translation2d[] getBallPositions(){
        return new Translation2d[] {
            new Translation2d(0.106, -0.01),
            new Translation2d(-0.474, 0.139),
            new Translation2d(0.043, 0.183)
        };
    }

}
