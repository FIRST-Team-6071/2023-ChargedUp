// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveSetDistance extends PIDCommand {
  DriveSubsystem m_Drive;
  double amountToMove;
  /** Creates a new MoveSetDistance. */
  public MoveSetDistance(DriveSubsystem p_Drive, double p_amountToMove) {
    super(
        // The controller that the command will use
        new PIDController(6, 2, 0),
        // This should return the measurement
        () -> p_Drive.GetRobotPoseX(),
        // This should return the setpoint (can also be a constant)
        () -> p_amountToMove,
        // This uses the output
        output -> {
          // Use the output here
          p_Drive.drive(MathUtil.clamp(output, -0.4, 0.4), 0, 0, false, false);
        });

    p_Drive.resetOdometry(new Pose2d());


    amountToMove = p_amountToMove;
    m_Drive = p_Drive;

    addRequirements(p_Drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Drive.GetRobotPoseX() > (amountToMove - .50) && m_Drive.GetRobotPoseX() < (amountToMove + .50)) {
      m_Drive.resetOdometry(new Pose2d());
      return true;
    }
    return false;
  }
}
