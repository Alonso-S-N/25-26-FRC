package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionValidator {

  private Pose2d lastPose = null;

  private static final double MAX_TRANSLATION_JUMP = 0.4; // m
  private static final double MAX_ROTATION_JUMP = Math.toRadians(20);

  public boolean isValid(Pose2d pose) {

    if (lastPose == null) {
      lastPose = pose;
      return true;
    }

    double translationDelta =
        pose.getTranslation()
            .getDistance(lastPose.getTranslation());

    double rotationDelta =
        Math.abs(
            pose.getRotation()
                .minus(lastPose.getRotation())
                .getRadians()
        );

    lastPose = pose;

    return translationDelta < MAX_TRANSLATION_JUMP &&
           rotationDelta < MAX_ROTATION_JUMP;
  }

  public void reset() {
    lastPose = null;
  }
}
