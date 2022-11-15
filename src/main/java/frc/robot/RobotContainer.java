// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.TurnTurret;
import frc.robot.commands.Vision;
import frc.robot.controls.ControlMap;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public Drivetrain m_drivetrain = new Drivetrain();
  private TurnTurret m_left = new TurnTurret(m_drivetrain, -0.05);
  private TurnTurret m_right = new TurnTurret(m_drivetrain, 0.05);
  private TurnTurret m_stop = new TurnTurret(m_drivetrain, 0);
  private Vision m_vision = new Vision(m_drivetrain);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_chooser.addOption("Turn Left", m_left);
    m_chooser.addOption("Turn Right", m_right);
    m_chooser.addOption("Stop look both ways, passenger has right of way", m_stop);
    m_chooser.addOption("Vision", m_vision);
    SmartDashboard.putData(m_chooser);

    turretButtonBindings();
  }

  private void turretButtonBindings() {
    ControlMap.GUNNER_A.whenPressed(m_left);
    ControlMap.GUNNER_B.whenPressed(m_right);
    ControlMap.GUNNER_X.whenPressed(new TurnTurret(m_drivetrain, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
