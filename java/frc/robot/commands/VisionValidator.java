package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;

public class VisionValidator {

  private Pose2d lastPose = new Pose2d();
  public VisionValidator() {
  }

  public boolean isValid(Pose2d pose){
    if (lastPose == null){
      lastPose = pose;
      return true;
    }
     double delta = pose.getTranslation()
                    .getDistance(lastPose.getTranslation());

                    lastPose = pose;

                    return delta < 0.5;
  }

  public void reset() {
    lastPose = null;
  }
}
