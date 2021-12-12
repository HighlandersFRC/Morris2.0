package frc.robot.commands;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.tools.math.Vector;

import org.json.JSONArray;
import org.json.JSONObject;

public class ContinuousAccelerationInterpolation extends CommandBase {
    /** Creates a new ContinuousAccelerationInterpolation. */

    private Drive drive;

    private double currentX = 0;
    private double currentY = 0;
    private double currentTheta = 0;

    private double initTime;
    private double currentTime;
    private double previousTime;
    private double timeDiff;

    private double previousX = 0;
    private double previousY = 0;
    private double previousTheta = 0;

    private double currentXVelocity = 0;
    private double currentYVelocity = 0;
    private double currentThetaVelocity = 0;

    private JSONArray pathPointsJSON;

    public ContinuousAccelerationInterpolation(Drive drive, JSONArray path) {
      this.drive = drive;
      this.pathPointsJSON = path;
      addRequirements(drive);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      initTime = Timer.getFPGATimestamp();
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentX = drive.getOdometryX();
    currentY = drive.getOdometryY();
    currentTheta = drive.getOdometryAngle();
    currentTime = Timer.getFPGATimestamp() - initTime;
    timeDiff = currentTime - previousTime;
    // System.out.println(currentTime);

    currentXVelocity = (currentX - previousX) * 1/timeDiff;
    currentYVelocity = (currentY - previousY) * 1/timeDiff;
    currentThetaVelocity = (currentTheta - previousTheta) * 1/timeDiff;

    // System.out.println()

    currentX = 0;
    currentY = 0;
    currentXVelocity = 0;
    currentYVelocity = 0;

    drive.constantAccelerationInterpolation(currentX, currentY, currentTheta, currentXVelocity, currentYVelocity, currentThetaVelocity, currentTime, timeDiff, pathPointsJSON);
    
    previousX = currentX;
    previousY = currentY;
    previousTheta = currentTheta;
    previousTime = currentTime;
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