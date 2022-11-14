// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ControlMap {
  private static int GUNNER_PORT = 0;
    
  public static Joystick gunner = new Joystick(GUNNER_PORT);
  public static JoystickButton GUNNER_A = new JoystickButton(gunner, 1); //A
  public static JoystickButton GUNNER_B = new JoystickButton(gunner, 2); //B
  public static JoystickButton GUNNER_X = new JoystickButton(gunner, 3); //X
  // Called when the command is initially scheduled.
}
