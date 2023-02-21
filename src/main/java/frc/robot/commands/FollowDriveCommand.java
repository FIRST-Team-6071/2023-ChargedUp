// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowDriveCommand extends PIDCommand {
  DriveSubsystem m_DriveSubsystem;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  /** Creates a new FollowDriveCommand. */
  public FollowDriveCommand(DriveSubsystem p_DriveSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(2, 0, 0.1),

        // This should return the measurement
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0),

        // This should return the setpoint (can also be a constant)
        () -> 0,

        // This uses the output
        output -> {
          output = MathUtil.clamp(output / 100, -.5, .5);
          p_DriveSubsystem.drive(0, output, 0, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = p_DriveSubsystem;
    addRequirements(p_DriveSubsystem);
    
    // Configure additional PID options by calling `getController` here.

     // Set the controller to be continuous (because it is an angle controller)
     getController().enableContinuousInput(-180, 180);

     // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
     // setpoint before it is considered as having reached the reference
     getController().setTolerance(0, 2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
