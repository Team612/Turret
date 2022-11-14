// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Vision extends CommandBase {
  /*
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("photonvision");

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  double forwardSpeed;
  double rotationSpeed;

  // Drive motors
  PWMVictorSPX leftMotor = new PWMVictorSPX(0);
  PWMVictorSPX rightMotor = new PWMVictorSPX(1);
  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
  */
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController rotationController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  private double rotationSpeed = 0;
  private final Drivetrain m_drive;

  public Vision(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();
    System.out.println(camera.getLatestResult().getBestTarget().getYaw());
      //System.out.println(result.hasTargets());
      if(result.hasTargets()){
        rotationSpeed = -rotationController.calculate(result.getBestTarget().getYaw(), 0);
        System.out.println(rotationSpeed);

        double range = (Constants.CAMERA_HEIGHT_METERS - Constants.TARGET_HEIGHT_METERS)/ 
        Math.tan(Constants.CAMERA_PITCH_RADIANS + Units.degreesToRadians(result.getBestTarget().getPitch()));

        // forwardSpeed = forwardController.calculate(range, Constants.GOAL_RANGE_METERS);
        // System.out.println(forwardSpeed);
      }
      else{
        rotationSpeed = 0;
        // forwardSpeed = 0;
      }
    m_drive.turn(rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
