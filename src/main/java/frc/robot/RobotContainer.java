// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ExtendToPoint;
import frc.robot.commands.MoveSetDistance;
import frc.robot.commands.TiltToPoint;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ArmExtensionSubsystem m_ArmExt = new ArmExtensionSubsystem();
    private final ArmSubsystem m_Arm = new ArmSubsystem();
    private final PneumaticsSubsystem m_Pneumatics = new PneumaticsSubsystem();
    private final ShuffleboardSubsystem m_ShuffleboardSubsystem = new ShuffleboardSubsystem(m_Arm, m_robotDrive,
            m_ArmExt);

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_armController = new CommandXboxController(OIConstants.kArmControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        CameraServer.startAutomaticCapture(0);
        CameraServer.startAutomaticCapture(1);

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                MathUtil.applyDeadband(m_driverController.getLeftY(), 0.06),
                                MathUtil.applyDeadband(m_driverController.getLeftX(), 0.06),
                                MathUtil.applyDeadband(m_driverController.getRightX(), 0.06),
                                true, true),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Driver Controls
        new JoystickButton(m_driverController, Button.kA.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

        new JoystickButton(m_driverController, Button.kLeftStick.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.zeroHeading(),
                        m_robotDrive));

        new JoystickButton(m_driverController, Button.kX.value)
                .onTrue(m_robotDrive.ToggleSlowDriveMode(false));

        new JoystickButton(m_driverController, Button.kB.value)
                .onTrue(m_robotDrive.ToggleFastDriveMode());

        // Arm Controls
        m_armController.rightTrigger()
                .onTrue(new RunCommand(
                        () -> m_ArmExt.Retract(),
                        m_Arm))
                .onFalse(new RunCommand(
                        () -> m_ArmExt.StopExtension(),
                        m_Arm));

        m_armController.leftTrigger()
                .onTrue(new RunCommand(
                        () -> m_ArmExt.Extend(),
                        m_Arm))
                .onFalse(new RunCommand(
                        () -> m_ArmExt.StopExtension(),
                        m_Arm));

        // Arm Controls
        m_armController.povUp()
                .onTrue(m_Arm.TiltUp());

        m_armController.povDown()
                .onTrue(m_Arm.TiltDown());

        m_armController.povLeft()
                .onTrue(new AutoBalance(m_robotDrive));

        m_armController.a()
                .onTrue(m_Pneumatics.openClaw());

        m_armController.b()
                .onTrue(m_Pneumatics.closeClaw());

        m_armController.y()
                .onTrue(new ExtendToPoint(Constants.Arm.Extension.Encoder.k_Min, m_ArmExt)
                        .andThen(new TiltToPoint(Constants.Arm.Tilt.Encoder.k_Min, m_Arm)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // 0.1529
        return new ExtendToPoint(Constants.Arm.Extension.Encoder.k_Min, m_ArmExt)
                .andThen(m_Pneumatics.closeClaw())
                .andThen(new TiltToPoint(0.0420, m_Arm))

                .andThen(new TiltToPoint(0.1529, m_Arm).alongWith(
                        new ExtendToPoint(Constants.Arm.Extension.Encoder.k_Max, m_ArmExt)))

                .andThen(m_Pneumatics.openClaw())

                .andThen(new TiltToPoint(Constants.Arm.Tilt.Encoder.k_Min, m_Arm).alongWith(
                        new ExtendToPoint(Constants.Arm.Extension.Encoder.k_Min, m_ArmExt)))

                .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
                .andThen(new MoveSetDistance(m_robotDrive, 4.67))
                .andThen(new MoveSetDistance(m_robotDrive, -3))
                .andThen(new WaitCommand(.5))
                .andThen(new AutoBalance(m_robotDrive))
                .andThen(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

    }
}
