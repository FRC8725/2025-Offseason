package frc.robot.lib.math;

import org.apache.commons.math3.geometry.euclidean.threed.Plane;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoseTransform {
    private final Vector3D xAxis;
    private final Vector3D yAxis;
    private final Vector3D robotToCamera;
    @SuppressWarnings("deprecation")
    private final Plane aprilTagPlane = new Plane(new Vector3D(0.0, 0.0, 0.311), new Vector3D(0.0, 0.0, 1.0));

    public PoseTransform(Vector3D robotToCamera) {
        this.robotToCamera = robotToCamera;
        this.xAxis = new Vector3D(0.0, 1.0, 0.0);
        this.yAxis = new Vector3D(0.0, 0.0, 1.0);
    }

    @SuppressWarnings("deprecation")
    public Translation3d getTransform(Transform3d cameraToTag, double pitch, double yaw) {
        Vector3D tagVec = new Vector3D(cameraToTag.getX(), cameraToTag.getY(), cameraToTag.getZ());
        Rotation3d tagRot = cameraToTag.getRotation();
        double tagRotate = tagRot.getZ();

        Rotation xRot = new Rotation(this.yAxis, yaw + tagRotate, RotationConvention.VECTOR_OPERATOR);
        Rotation yRot = new Rotation(this.xAxis, pitch, RotationConvention.VECTOR_OPERATOR);
        Vector3D xVector = xRot.applyTo(tagVec);
        Vector3D yVector = yRot.applyTo(tagVec);

        Plane xPlane = new Plane(this.robotToCamera, this.robotToCamera.add(xVector), this.robotToCamera.add(this.yAxis));
        Plane yPlane = new Plane(this.robotToCamera, this.robotToCamera.add(yVector), this.robotToCamera.add(this.xAxis));
        Vector3D intersect = Plane.intersection(xPlane, yPlane, aprilTagPlane);

        return new Translation3d(intersect.getX(), intersect.getY(), intersect.getZ());
    }
}
