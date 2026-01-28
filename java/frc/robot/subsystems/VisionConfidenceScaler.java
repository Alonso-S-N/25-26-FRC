package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConfidenceScaler {
	private static final double MAX_STD = 1.2;
    private static final double MIN_STD = 0.05;

      public static Matrix<N3, N1> computeStdDevs(
      double distance,
      ChassisSpeeds speeds,
      double ta
  ) { 
      double confidence = 1.0;
     
     confidence *= MathUtil.clamp(1.0 - distance / 5.0, 0.2, 1.0);

    confidence *= MathUtil.clamp(
        1.0 - Math.abs(speeds.omegaRadiansPerSecond) / 3.0,
        0.3,
        1.0
    );

    confidence *= MathUtil.clamp(ta / 1.5, 0.3, 1.0);

    double std = MathUtil.interpolate(
        MAX_STD,
        MIN_STD,
        confidence
    );

    return VecBuilder.fill(std, std, 9999);
  }
}
