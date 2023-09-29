package frc.robot.Subsystems;

import frc.robot.MathStuff;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveModuleNeo extends SubsystemBase{
    private CANSparkMax drive;
    private CANSparkMax steer;
    private CANCoder encoder;
    private double offset;

    private double dM;
    private double sM;

    // m/s, rotation2d
    private SwerveModuleState targetState = new SwerveModuleState();

    // pass in a ff+pid object or something, o is offset in radians
    public SwerveModuleNeo(int driveID, int steerID, int encoderID, double o, double driveMult, double steerMult) {
        drive = new CANSparkMax(driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
        steer = new CANSparkMax(steerID, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = new CANCoder(encoderID);

        offset = o;


        // multiplier prob just for reversing
        dM = driveMult;
        sM = steerMult;
    }

    public void go() {
        //System.out.println("speed: "+targetState.speedMetersPerSecond);
        // get to the set positions 
            
        //System.out.println(targetState.angle.getDegrees()+", "+getRotation().getDegrees()+", "+sM);
        double steerA = MathStuff.subtract(targetState.angle, getRotation()).getRotations()*sM;
        double steerB = Math.signum(steerA)*0.008;
        steer.set(steerA+steerB);

        //if()
        double driveA = targetState.speedMetersPerSecond*dM*0.8;
        double driveB = Math.signum(driveA)*0.03;
        drive.set(driveA+driveB);
    }

    public void setSpeeds(double d, double s) {
        drive.set(d*dM);
        steer.set(s*sM);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drive.getEncoder().getPosition(), 
            new Rotation2d(encoder.getAbsolutePosition() * 0.0174533) // 2048 ticks to radians is 2pi/2048
        );
    }
    public double getDriveVelocity() { //rpms default supposedy, actual drive speed affected by gear ratio and wheel circumfernce
        return drive.getEncoder().getVelocity(); //*gearratio*circumference=m/s except needs units adjustment
    }
    public double getSteerVelocity() { //rpms default, affected by gear ratio
        return steer.getEncoder().getVelocity(); // /gearratio=rpms of the wheel spinning
    }
    public Rotation2d getRotation() {
        return new Rotation2d((encoder.getAbsolutePosition() + offset + 90) * 0.0174533);
    }

    // -1 to 1
    /*public void setSpeeds(double driveSpeed, double turnSpeed) {

        // check controlmodes for maybe useful or cool stuff
        drive.set(TalonFXControlMode.PercentOutput, driveSpeed);
        drive.set(TalonFXControlMode.PercentOutput, turnSpeed);
    }*/
    
    // default use optimization
    public void setTargetState(double metersPerSecond, Rotation2d angle) {
        targetState = new SwerveModuleState(metersPerSecond, angle);
        SwerveModuleState.optimize(targetState, getRotation());
    }
    public void setTargetState(double metersPerSecond, Rotation2d angle, boolean optimize) {
        targetState = new SwerveModuleState(metersPerSecond, angle);
        if(optimize) SwerveModuleState.optimize(targetState, getRotation());
    }
    public void setTargetState(SwerveModuleState target, boolean optimize) {
        targetState = target;
        if(optimize) targetState = SwerveModuleState.optimize(targetState, getRotation());
    }
}
