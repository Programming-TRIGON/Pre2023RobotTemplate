package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.utils.AprilTagsLocations;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.trigon.robot.utils.AprilTagsLocations.AprilTag;

public class UpdatePoseFromTagCommand extends CommandBase {
    final PhotonCamera camera;
    final Pose3d cameraPose;
    PhotonPipelineResult lastResult = new PhotonPipelineResult();

    final Field2d field2d;

    public UpdatePoseFromTagCommand(PhotonCamera camera, Pose3d cameraPose) {
        this.camera = camera;
        this.cameraPose = cameraPose;
        field2d = (Field2d) SmartDashboard.getData("Field");

        putTagsOnField();
    }

    private void putTagsOnField() {
        for(AprilTag tag : AprilTagsLocations.locations)
            field2d.getObject("Tag " + tag.id).setPose(tag.pose.toPose2d());
    }

    @Override
    public void execute() {
        field2d.setRobotPose(Swerve.getInstance().getPose());
        PhotonPipelineResult latest = camera.getLatestResult();
        if(latest == null || latest.equals(lastResult))
            return;
        lastResult = latest;
        addMeasurementsFromResult(latest);
    }

    private void addMeasurementsFromResult(PhotonPipelineResult latest) {
        for(PhotonTrackedTarget target : latest.getTargets()) {
            AprilTag tag = AprilTagsLocations.getTag(target.getFiducialId());
            if(tag == null)
                continue;
            Pose2d robotPose = calculatePoseFromTag(target, tag.pose);
            field2d.getObject("Vision based on tag " + tag.id).setPose(robotPose);
            Swerve.getInstance().addVisionMeasurement(robotPose, Timer.getFPGATimestamp() - Units.millisecondsToSeconds(latest.getLatencyMillis()));
        }
    }

    private Pose2d calculatePoseFromTag(PhotonTrackedTarget target, Pose3d tagPose) {
        Transform3d photonRes = target.getBestCameraToTarget();
        var inverted = new Transform3d(photonRes.inverse().getTranslation(), photonRes.inverse().getRotation());
        var adjusted = new Pose2d(new Translation2d(inverted.getX(), -inverted.getY()), inverted.getRotation().toRotation2d().unaryMinus());
        return tagPose.toPose2d().transformBy(new Transform2d(adjusted.getTranslation(), adjusted.getRotation()));
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
