package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSub;

public class resetPoseByTag extends InstantCommand {

  private final SwerveSub swerve;

  public resetPoseByTag(SwerveSub swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.getMegaTagPose().ifPresentOrElse(
        pose -> {
          swerve.resetOdometry(pose);
          System.out.println("✅ Pose resetada pelo MegaTag: " + pose);
        },
        () -> System.out.println("❌ MegaTag sem pose válida")
    );
  }
}
