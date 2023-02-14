// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Joystick driver = new Joystick(0);
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton zeroGyro = 
    new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric =
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final Swerve s_Swerve = new Swerve();
  private final ArmSubsystem Armm = new ArmSubsystem();
  private final JoystickButton Intake = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton Low = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton Medium = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton High = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        () -> robotCentric.getAsBoolean()));
    Armm.setDefaultCommand(
      new Arm(
        Armm, 
        () -> Intake.getAsBoolean(), 
        () -> Low.getAsBoolean(), 
        () -> Medium.getAsBoolean(), 
        () -> High.getAsBoolean()));
    // Configure the button bindings
    configureButtonBindings();
    
  }
  //robotCentric.get();
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }
}
