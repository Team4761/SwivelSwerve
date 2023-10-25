// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.SwerveGoCartesianF;
import frc.robot.Subsystems.SwerveDrive;

//import frc.robot.Subsystems.SwerveDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static XboxController xbox = new XboxController(0);
  public static SwerveDrive swerveDrive = SwerveDrive.getInstance();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    
    // x should be sideways +x left
    CommandScheduler.getInstance().schedule(new SwerveGoCartesianF(swerveDrive, new Translation2d(20, 20)));
  }

  /** This function is called periodically during autonomous. */

  @Override
  public void autonomousPeriodic() {

    CommandScheduler.getInstance().run();
    
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  
  
  public static double deadzone (double value, double deadzone) {
    if (Math.abs(value) > deadzone) {
        if (value > 0.0) {
            return (value - deadzone) / (1.0 - deadzone);
        } else {
            return (value + deadzone) / (1.0 - deadzone);
        }
    } else {
        return 0.0;
    }

}
  @Override
  public void teleopPeriodic() {
    //swerveDrive.swerveDriveR(xbox.getLeftY(), -xbox.getLeftX(), xbox.getRightX());

    //inverse of magnitude to normalize x and y
    double xyCof = 0.75/Math.max(0.001, Math.sqrt(Math.pow(deadzone(xbox.getLeftX(), 0.1), 2)+Math.pow(deadzone(xbox.getLeftY(), 0.1), 2)));
    swerveDrive.swerveDriveF(xyCof*deadzone(xbox.getLeftX(), 0.1)*(xbox.getLeftTriggerAxis()+xbox.getRightTriggerAxis()), -xyCof*deadzone(xbox.getLeftY(), 0.1)*(xbox.getLeftTriggerAxis()+xbox.getRightTriggerAxis()), -deadzone(xbox.getRightX(), 0.08));
    
    double input = 1;
    if(input>1) {
      //do whatever in here
    } else if (input>-1) {
      //runs if input<=1
    }

    if(xbox.getXButtonPressed()) {
      swerveDrive.zeroGyro();
    }
    if(xbox.getYButtonPressed()) {
      swerveDrive.resetPose();
    }
    CommandScheduler.getInstance().run();
    //swerveDrive.periodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
