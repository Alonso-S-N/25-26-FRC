package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class GoToPoseCommand {

  public static Command go(Pose2d targetPose) {
    return AutoBuilder.pathfindToPose(
        targetPose, 
        new PathConstraints(
            3.0,
            1.5,
            4,
            4
        ),
        0.0
    ).withTimeout(2)
    .withName("PathfindToPose");

  }
}
