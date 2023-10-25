package frc.robot.Subsystems;

import frc.robot.Subsystems.SwerveModuleTalon;
import frc.robot.Subsystems.SwerveModuleNeo;

import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveDrive extends SubsystemBase {
    // inches rn
    private static final SwerveDrive INSTANCE = new SwerveDrive(new Translation2d(-12.25, 12.25), new Translation2d(12.25, 12.25), new Translation2d(-12.25, -12.25), new Translation2d(12.25, -12.25));

    SwerveModuleState[] targetStates = new SwerveModuleState[4];

    // motors offset in degrees && i think negative is ccw
    private SwerveModuleNeo m_frontLeftModule  = new SwerveModuleNeo(3 , 1 , 2 , -98.5, -1.0,  1.0);
    private SwerveModuleNeo m_frontRightModule = new SwerveModuleNeo(4 , 6 , 4 , -6, -1.0, -1.0);
    private SwerveModuleNeo m_backLeftModule   = new SwerveModuleNeo(7 , 8 ,3 , 112, -1.0, -1.0);
    private SwerveModuleNeo m_backRightModule  = new SwerveModuleNeo(5 , 2 , 1 , -53, -1.0,  1.0);

    private SwerveModulePosition[] m_swervePositions= new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(), 
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(), 
        m_backRightModule.getPosition()
    };

    private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    private Rotation2d gyroOffset = new Rotation2d();

    private SwerveDriveKinematics m_kinematics;

    private SwerveDriveOdometry m_odometry;
    private Pose2d m_pose;

    private Rotation2d pointDir = new Rotation2d();


    // positions of the wheels relative to center (meters?)
    public SwerveDrive (Translation2d fL, Translation2d fR, Translation2d bL, Translation2d bR) {
        m_kinematics = new SwerveDriveKinematics(fL, fR, bL, bR);
        m_pose = new Pose2d();
        m_odometry =  new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), m_swervePositions, m_pose);
        targetStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getGyroRotation()));
    }
    
    public static SwerveDrive getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
        m_swervePositions= new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(), 
            m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(), 
            m_backRightModule.getPosition()
        };
        
        // update pose
        m_pose = m_odometry.update(
            m_gyro.getRotation2d(),
            m_swervePositions
        );

        //forward is -y left is +x

        SmartDashboard.putNumber("Odometry x", m_pose.getX());
        SmartDashboard.putNumber("Odometry y", m_pose.getY());

        
        SmartDashboard.putNumber("Front Left Rot", m_frontLeftModule.getRotation().getDegrees());
        SmartDashboard.putNumber("Front Right Rot", m_frontRightModule.getRotation().getDegrees());
        SmartDashboard.putNumber("Back Left Rot", m_backLeftModule.getRotation().getDegrees());
        SmartDashboard.putNumber("Back Right Rot", m_backRightModule.getRotation().getDegrees());

        
        SmartDashboard.putNumber("Front Left Target", targetStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Front Right Target", targetStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Back Left Target", targetStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Back Right Target", targetStates[3].angle.getDegrees());

        SmartDashboard.putNumber("Gyro Angle", m_gyro.getRotation2d().minus(gyroOffset).getDegrees());

        // set the stuff
        m_frontLeftModule .setTargetState(targetStates[0], true);
        m_frontRightModule.setTargetState(targetStates[1], true);
        m_backLeftModule  .setTargetState(targetStates[2], true);
        m_backRightModule .setTargetState(targetStates[3], true);


        m_frontLeftModule.go();
        m_frontRightModule.go();
        m_backLeftModule.go();
        m_backRightModule.go();



        //System.out.println("Back Left target/measure: "+targetStates[2].angle.getDegrees()+" | "+m_backLeftModule.getRotation().getDegrees());
        //System.out.println("Ecoders: "+m_frontLeftModule.getRotation().getDegrees()+", "+m_frontRightModule.getRotation().getDegrees()+", "+m_backLeftModule.getRotation().getDegrees()+", "+m_backRightModule.getRotation().getDegrees());
    }
    
    // drives
    // try applying acceleration cap to inputs in drives instead of on wheels

    private int lastDone = 10;
    // field oriented, m/s, m/s, rad/s or something, i think x is forward
    public void swerveDriveF(double speedX, double speedY, double speedRot) {
        SmartDashboard.putNumber("Gyro Target", pointDir.getDegrees());



        // input squaring
        double squareFactor = Math.sqrt(speedX*speedX+speedY*speedY);
        speedX*=squareFactor;
        speedY*=squareFactor;


        if(lastDone==0) {
            pointDir = getGyroRotation();
        }
        // if not turning do lock on
        if (speedRot == 0) {
            double rotP = pointDir.minus(getGyroRotation()).getDegrees()*0.001; // proportional
            //rotP += Math.signum(rotP)*0.00005;
            if (Math.abs(rotP)<0.001 || lastDone>0) rotP=0;
            targetStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rotP, getGyroRotation()));
            
            lastDone--;
        } else { // if turning dont proportional
            // need 
            speedRot=Math.signum(speedRot)*speedRot*speedRot;

            targetStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedRot*0.09, getGyroRotation()));
            pointDir = getGyroRotation();

            lastDone = 10;
        }
        
    }
//for on the go field oriented and stuff
    public void zeroGyro() {
        pointDir = pointDir.minus(getGyroRotation());
        gyroOffset = m_gyro.getRotation2d();
    }
    
    public void resetPose() {
        m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), m_swervePositions);
        m_pose = new Pose2d();
    }

    // robot oriented, m/s, m/s, rad/s or something
    public void swerveDriveR(double speed, double strafe, double speedRot) {

        targetStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(speed, strafe, speedRot));
    }
    // car, m/s, degrees    
    public void carDrive(double speed, double turn) {
        turn = MathUtil.clamp(turn, -90, 90);
        
        // do the optimize thing
        
        targetStates[0] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));
        targetStates[1] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));
        targetStates[2] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));
        targetStates[3] = new SwerveModuleState(speed, new Rotation2d(turn*0.0174533));

    }

    // stuff
    // degrees
    public double getGyroAngle() {
        return m_gyro.getRotation2d().minus(gyroOffset).getDegrees();
    }
    public Rotation2d getGyroRotation() {
        return m_gyro.getRotation2d().minus(gyroOffset);
    }
    
    public Pose2d getPose() {
        return m_pose;
    }

    public void stop() {
        m_frontLeftModule.setSpeeds(0, 0);
        m_frontRightModule.setSpeeds(0, 0);
        m_backLeftModule.setSpeeds(0, 0);
        m_backRightModule.setSpeeds(0, 0);
    }
}
