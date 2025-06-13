package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Helpers;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.Robot;
import frc.robot.Robot.RobotMode;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.RAROdometry;

import java.util.ArrayList;

public class DetectionIOLimelight {
	private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
	private ArrayList<Coral> tracker = new ArrayList<Coral>();
  private String mLimelightName = "limelight_gp";
	private final NetworkTable visTable = ntInstance.getTable("SmartDashboard/Detection");
	private final StructPublisher<Pose2d> closestCoralPose =
			visTable.getStructTopic("BestCoralPose", Pose2d.struct).publish();
	private final StructPublisher<Translation2d> closestCoralTranslation = visTable.getStructTopic(
					"BestCoralTranslation", Translation2d.struct)
			.publish();
	protected StructPublisher<Pose2d> aprilTagPose = NetworkTableInstance.getDefault()
			.getTable("SmartDashboard/Detection/AprilTagPose")
			.getStructTopic("", Pose2d.struct)
			.publish();

	class Coral {
		Pose2d coralPose;
		Translation2d coralTranslation;
		double detectionTime;

		public Coral(Pose2d coralPose, Translation2d coralTranslation, double detectionTime) {
			this.coralPose = coralPose;
			this.coralTranslation = coralTranslation;
			this.detectionTime = detectionTime;
		}
	}

	public void update() {
    Translation2d base = RAROdometry.getInstance().getPose().getTranslation();
    RawDetection[] all = LimelightHelpers.getRawDetections(mLimelightName);
    Translation2d bestTranslation = null;
    Pose2d bestCoralPose = null;
    double now = Timer.getFPGATimestamp();
    tracker.removeIf((coral) -> now - coral.detectionTime > 0.2);

    while (tracker.size() > 20) {
      tracker.remove(0);
    }

    for (RawDetection detection : all) {
      if (detection.classId == 0) continue;
      double tx = detection.txnc;
      double ty = detection.tync;
      Translation2d coralTranslation = calcDistToCoral(tx, ty);
      Pose2d coralPose = RAROdometry.getInstance().getPose().transformBy(new Transform2d(coralTranslation, new Rotation2d()));
      if (Helpers.isPoseOutsideField(coralPose)) {
        // LogUtil.recordPose2d(config.name + "Last Coral Pose Outside Field", coralPose);
        DriverStation.reportWarning("Coral is found outside of play area!", false);
        continue;
      }
      tracker.add(new Coral(coralPose, coralTranslation, now));
    }

    for (Coral coral : tracker) {
      if (bestTranslation == null
          || bestCoralPose.getTranslation().getDistance(base)
              > coral.coralPose.getTranslation().getDistance(base)) {
        bestTranslation = coral.coralTranslation;
        bestCoralPose = coral.coralPose;
      }
    }

    if (bestCoralPose != null) {
      closestCoralPose.set(bestCoralPose);
      closestCoralTranslation.set(bestTranslation);
    }
	}

	public Pose2d getCoralPose(Translation2d base) {
		Translation2d bestTranslation = null;
		Pose2d bestCoralPose = null;
		for (Coral coral : tracker) {
			if (bestTranslation == null
					|| bestCoralPose.getTranslation().getDistance(base)
							> coral.coralPose.getTranslation().getDistance(base)) {
				bestTranslation = coral.coralTranslation;
				bestCoralPose = coral.coralPose;
			}
		}
		return bestCoralPose; // will return null if no coral
	}

	public boolean txComplete(double tx) {
		return Util.epsilonEquals(tx, 0, 4);
	}

	public int coralCount() {
		return LimelightHelpers.getTargetCount(mLimelightName);
	}

	public Translation2d calcDistToCoral(double tx, double ty) {
		double totalAngleY = Units.degreesToRadians(-ty);
		Distance distAwayY = config.robotToCameraOffset.getMeasureZ()
				.minus(RobotConstants.robotConfig.Vision.kCoralRadius)
				.div(Math.tan(totalAngleY)); // robot x

		Distance distHypotenuseYToGround = BaseUnits.DistanceUnit.of(Math.hypot(
				distAwayY.in(BaseUnits.DistanceUnit),
				RobotConstants.robotConfig.Vision.kMeasureZ//config.robotToCameraOffset.getMeasureZ()
				.minus(RobotConstants.robotConfig.Vision.kCoralRadius)
				.in(BaseUnits.DistanceUnit)));

		double totalAngleX = Units.degreesToRadians(-tx)
				+ RobotConstants.robotConfig.Vision.kRotZ;//config.robotToCameraOffset.getRotation().getZ();

		Distance distAwayX = distHypotenuseYToGround.times(Math.tan(totalAngleX)); // robot y

    //TODO: AKit
		// SmartDashboard.putNumber(config.name + "/tx", tx);
		// SmartDashboard.putNumber(config.name + "/ty", ty);
		// SmartDashboard.putNumber(config.name + "/Distance Away Y", distAwayY.in(edu.wpi.first.units.Units.Meters));
		// SmartDashboard.putNumber(config.name + "/Distance Away X", distAwayX.in(edu.wpi.first.units.Units.Meters));
		// SmartDashboard.putNumber(
		// 		config.name + "/Distance Away Hyp ", distHypotenuseYToGround.in(edu.wpi.first.units.Units.Meters));

		return new Translation2d(distAwayY, distAwayX);
	}

	public void setPipeline(int index) {
		pipelineToSet = index;
		LimelightHelpers.setPipelineIndex(mLimelightName, index);
	}
}