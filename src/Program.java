
public class Program 
{
	public static void main(String[] args)
	{
		System.out.println(Math.sin(0.087266));
		System.out.println(Math.sin(5));
		RigidTransform2d prev = new RigidTransform2d(new Translation2d(1,1), Rotation2d.fromDegrees(45));
		RigidTransform2d cur = generateOdometryFromSensors(prev, 1.414213562 ,0, Rotation2d.fromDegrees(100));
		
		System.out.println(cur.getTranslation().x_);
		System.out.println(cur.getTranslation().y_);
		System.out.println(cur.getRotation().getDegrees());
		
		System.out.println(cur.getTranslation().x_ + 1.414213562 * Math.cos(Math.toRadians(100)));
		System.out.println(cur.getTranslation().y_ + 1.414213562 * Math.sin(Math.toRadians(100)));
		System.out.println(50);


	}
	
    public static RigidTransform2d generateOdometryFromSensors(RigidTransform2d last, double left_encoder_delta_distance,
            double right_encoder_delta_distance, Rotation2d current_gyro_angle) 
    {
        RigidTransform2d last_measurement = last;
        
        return integrateForwardKinematics(last_measurement, left_encoder_delta_distance,
                right_encoder_delta_distance, current_gyro_angle);
    }
    
    public static RigidTransform2d integrateForwardKinematics(RigidTransform2d current_pose, double left_wheel_delta,
            double right_wheel_delta, Rotation2d current_heading) {
        
    	System.out.println("degrees: " +  current_pose.getRotation().getDegrees());
    	System.out.println("degrees: " +  current_pose.getRotation().inverse().getDegrees());
    	System.out.println("degrees: " + current_heading.getDegrees());
    	System.out.println("degrees: " +  current_pose.getRotation().inverse().rotateBy(current_heading).getRadians());

        RigidTransform2d.Delta with_gyro = forwardKinematics(left_wheel_delta, right_wheel_delta,
                current_pose.getRotation().inverse().rotateBy(current_heading).getRadians());
        System.out.println("with: " + with_gyro.dx + ", " + with_gyro.dy + ", " + with_gyro.dtheta);
        RigidTransform2d a = RigidTransform2d.fromVelocity(with_gyro);
        System.out.println(current_pose.getTranslation().x_ + ", " + current_pose.getTranslation().y_ + ", " + current_pose.getRotation().getDegrees());
        System.out.println(a.getTranslation().x_ + ", " + a.getTranslation().y_ + ", " + a.getRotation().getDegrees());
        return current_pose.transformBy(a);
    }
    
    public static RigidTransform2d.Delta forwardKinematics(double left_wheel_delta, double right_wheel_delta,
            double delta_rotation_rads) {
        return new RigidTransform2d.Delta((left_wheel_delta + right_wheel_delta) / 2, 0, delta_rotation_rads);
    }

    
}
