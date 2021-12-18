package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.defaults.DriveDefault;
import frc.robot.tools.math.Vector;

public class Drive extends SubsystemBase {
    private final OI OI = new OI();

    // interpolation range
    private final double turnTimePercent = 0.8;

    private final double interpolationCorrection = 0.5;

    private final double cyclePeriod = 1.0/50.0;

    // creating all the falcons
    private final WPI_TalonFX leftForwardMotor = new WPI_TalonFX(3);
    private final WPI_TalonFX leftForwardAngleMotor = new WPI_TalonFX(4);
    private final WPI_TalonFX leftBackMotor = new WPI_TalonFX(5);
    private final WPI_TalonFX leftBackAngleMotor = new WPI_TalonFX(6);
    private final WPI_TalonFX rightForwardMotor = new WPI_TalonFX(1);
    private final WPI_TalonFX rightForwardAngleMotor = new WPI_TalonFX(2);
    private final WPI_TalonFX rightBackMotor = new WPI_TalonFX(7);
    private final WPI_TalonFX rightBackAngleMotor = new WPI_TalonFX(8);

    private Peripherals peripherals;

    private double adjustedX = 0.0;
    private double adjustedY = 0.0;

    private final double moduleXY = ((Constants.ROBOT_WIDTH)/2) - Constants.MODULE_OFFSET;

    // creating all the external encoders
    private CANCoder backRightAbsoluteEncoder = new CANCoder(4);
    private CANCoder frontLeftAbsoluteEncoder = new CANCoder(2);
    private CANCoder frontRightAbsoluteEncoder = new CANCoder(1);
    private CANCoder backLeftAbsoluteEncoder = new CANCoder(3);

    // creating each swerve module with angle and drive motor, module number(relation to robot), and external encoder
    private final SwerveModule leftFront = new SwerveModule(2, leftForwardAngleMotor, leftForwardMotor, 0, frontLeftAbsoluteEncoder);
    private final SwerveModule leftBack = new SwerveModule(3, leftBackAngleMotor, leftBackMotor, 0, backLeftAbsoluteEncoder);
    private final SwerveModule rightFront = new SwerveModule(1, rightForwardAngleMotor, rightForwardMotor, 0, frontRightAbsoluteEncoder);
    private final SwerveModule rightBack = new SwerveModule(4, rightBackAngleMotor, rightBackMotor, 0, backRightAbsoluteEncoder);

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(moduleXY, moduleXY);
    Translation2d m_frontRightLocation = new Translation2d(moduleXY, -moduleXY);
    Translation2d m_backLeftLocation = new Translation2d(-moduleXY, moduleXY);
    Translation2d m_backRightLocation = new Translation2d(-moduleXY, -moduleXY);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    // Creating my odometry object from the kinematics object. Here,
    // our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    SwerveDriveOdometry m_odometry; 
    Pose2d m_pose;

    double initAngle;
    double setAngle;
    double diffAngle;

    public Drive(Peripherals peripherals) {
        this.peripherals = peripherals;

        m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(Math.toRadians(peripherals.getNavxAngle())));
    }

    // get each external encoder
    public double getLeftForwardEncoder() {
        return leftFront.getAbsolutePosition();
    }

    public double getLeftBackEncoder() {
        return leftBack.getAbsolutePosition();
    }

    public double getRightForwardEncoder() {
        return rightFront.getAbsolutePosition();
    }

    public double getRightBackEncoder() {
        return rightBack.getAbsolutePosition();
    }

    // get Joystick adjusted y-value
    public double getAdjustedY(double originalX, double originalY){
        double adjustedY = originalY * Math.sqrt((1-(Math.pow(originalX, 2))/2));
        return adjustedY;
    }

    // get Joystick adjusted x-value
    public double getAdjustedX(double originalX, double originalY){
        double adjustedX = originalX * Math.sqrt((1-(Math.pow(originalY, 2))/2));
        return adjustedX;
    }

    // method run on robot initialization
    public void init() {
        leftFront.init();
        leftBack.init();
        rightBack.init();
        rightFront.init();
        peripherals.zeroNavx();

        setDefaultCommand(new DriveDefault(this));
    }

    public void autoInit(JSONArray pathPoints) {
        m_odometry.resetPosition(new Pose2d(new Translation2d(pathPoints.getJSONObject(0).getDouble("x"), pathPoints.getJSONObject(0).getDouble("y")), new Rotation2d()), new Rotation2d());
    }

    public double getOdometryX() {
        return m_odometry.getPoseMeters().getX();
    }

    public double getOdometryY() {
        return m_odometry.getPoseMeters().getY();
    }

    public double getOdometryAngle() {
        return m_odometry.getPoseMeters().getRotation().getRadians();
    }

    // method to actually run swerve code
    public void teleopDrive() {
        double turnLimit = 1;
        // this is correct, X is forward in field, so originalX should be the y on the joystick
        double originalX = -OI.getDriverLeftY();
        double originalY = -OI.getDriverLeftX();

        if(Math.abs(originalX) < 0.05) {
            originalX = 0;
        }
        if(Math.abs(originalY) < 0.05) {
            originalY = 0;
        }

        double turn = -OI.getDriverRightX() * (Constants.TOP_SPEED)/(Constants.ROBOT_RADIUS);
        double navxOffset = peripherals.getNavxAngle();
        double xPower = getAdjustedX(originalX, originalY);
        double yPower = getAdjustedY(originalX, originalY);

        double xSpeed = xPower * Constants.TOP_SPEED;
        double ySpeed = yPower * Constants.TOP_SPEED;

        Vector controllerVector = new Vector(xSpeed, ySpeed);

        m_pose = m_odometry.update(new Rotation2d(Math.toRadians(navxOffset)), leftFront.getState(Math.toRadians(navxOffset)), rightFront.getState(Math.toRadians(navxOffset)), leftBack.getState(Math.toRadians(navxOffset)), rightBack.getState(Math.toRadians(navxOffset)));

        leftFront.velocityDrive(controllerVector, turn, navxOffset);
        rightFront.velocityDrive(controllerVector, turn, navxOffset);
        leftBack.velocityDrive(controllerVector, turn, navxOffset);
        rightBack.velocityDrive(controllerVector, turn, navxOffset);

        // leftFront.testDrive();
        // rightFront.testDrive();
        // leftBack.testDrive();
        // rightBack.testDrive();

        rightFront.postDriveMotorTics();

        System.out.println(m_odometry.getPoseMeters());
        // System.out.println("Rate: " + peripherals.getNavxRate());
    }

    public void autoDrive(Vector velocityVector, double turnRadiansPerSec) {
        double navxOffset = peripherals.getNavxAngle();

        m_pose = m_odometry.update(new Rotation2d(Math.toRadians(-navxOffset)), leftFront.getState(Math.toRadians(-navxOffset)), rightFront.getState(Math.toRadians(-navxOffset)), leftBack.getState(Math.toRadians(-navxOffset)), rightBack.getState(Math.toRadians(navxOffset)));

        // System.out.println(m_odometry.getPoseMeters() + "Angle: " + getOdometryAngle());

        leftFront.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
        rightFront.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
        leftBack.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
        rightBack.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
    }

    public double safeDivision(double numerator, double denominator) {
        if(Math.abs(denominator) < 0.00001) {
            return 0.0;
        }
        double dividend = numerator/denominator;
        return dividend;
    }

    public boolean constantAccelerationInterpolation(double currentX, double currentY, double currentTheta, double currentXVelocity, double currentYVelocity, double currentThetaVelocity, double time, double timeSinceLastCycle, JSONArray pathPointsJSON) {
        JSONObject currentPoint;
        JSONObject nextPoint;
        JSONObject previousPoint;

        double[] returnArray = new double[3];
        int currentPointIndex = 0;
        double lowestTimeDiff = 0;

        double[] timeDiffArray = new double[pathPointsJSON.length()];

        for(int i = 0; i < pathPointsJSON.length(); i++) {
            timeDiffArray[i] = Math.abs(time - pathPointsJSON.getJSONObject(i).getDouble("time"));
        }
        for(int i = 0; i < timeDiffArray.length; i++) {
            if(i == 0) {
                lowestTimeDiff = timeDiffArray[0];
                currentPointIndex = 0;
            }
            else {
                if(timeDiffArray[i] < lowestTimeDiff) {
                    lowestTimeDiff = timeDiffArray[i];
                    currentPointIndex = i;
                }
            }
        }

        if(time > pathPointsJSON.getJSONObject(pathPointsJSON.length() - 1).getDouble("time")) {
            double velocityX = 0;
            double velocityY = 0;
            double thetaChange = 0;

            Vector velocityVector = new Vector(velocityX, velocityY);
            autoDrive(velocityVector, thetaChange);    
            return true;        
        }

        currentPoint = pathPointsJSON.getJSONObject(currentPointIndex);

        double currentPointTime = currentPoint.getDouble("time");

        double velocityX = 0;
        double velocityY = 0;
        double thetaChange = 0;

        if(currentPointIndex + 1 >= pathPointsJSON.length()) {
            previousPoint = pathPointsJSON.getJSONObject(currentPointIndex - 1);
            // System.out.println(Math.abs((currentPointTime - time)));
            // System.out.println(cyclePeriod);
            if(Math.abs((currentPointTime - time)) < cyclePeriod) {
                System.out.println("Less than cycle");
                velocityX = currentXVelocity;
                velocityY = currentYVelocity;
                thetaChange = currentThetaVelocity;
            }
            else {
                velocityX = (currentPoint.getDouble("x") - currentX)/(currentPointTime - time);
                velocityY = (currentPoint.getDouble("y") - currentY)/(currentPointTime - time);
                thetaChange = (currentPoint.getDouble("angle") - currentTheta)/(currentPointTime - time);
            }
            // velocityArray[0] = velocityX;
            // velocityArray[1] = velocityY;
            // velocityArray[2] = thetaChange;
            // return velocityArray;
            Vector velocityVector = new Vector(velocityX, velocityY);


            autoDrive(velocityVector, thetaChange);
            // System.out.println("CurrentPointTime: " + currentPointTime + " Time: " + time + " VelocityX: " + velocityVector.getI() + " VelocityY: " + velocityVector.getJ() + "Theta: " + currentTheta + " ThetaChange: " + thetaChange + " CurentX: " + currentX + " CurrentY: " + currentY);
            System.out.println("Time: " + time + " VelocityX: " + velocityVector.getI() + " VelocityY: " + velocityVector.getJ() + " ThetaChange: " + thetaChange + " CurentX: " + currentX + " CurrentY: " + currentY);
            
            return true;
        }
        else if(currentPointIndex - 1 < 0) {
            nextPoint = pathPointsJSON.getJSONObject(currentPointIndex + 1);
            double nextPointTime = nextPoint.getDouble("time");
            if(Math.abs((nextPointTime - time)) < cyclePeriod) {
                velocityX = currentXVelocity;
                velocityY = currentYVelocity;
                thetaChange = currentThetaVelocity;
            }
            else {
                velocityX = (nextPoint.getDouble("x") - currentX)/(nextPointTime - time);
                velocityY = (nextPoint.getDouble("y") - currentY)/(nextPointTime - time);
                thetaChange = (nextPoint.getDouble("angle") - currentTheta)/(nextPointTime - time);
            }
            // velocityArray[0] = velocityX;
            // velocityArray[1] = velocityY;
            // velocityArray[2] = thetaChange;
            // return velocityArray;
            Vector velocityVector = new Vector(velocityX, velocityY);

            autoDrive(velocityVector, thetaChange);
            System.out.println("Time: " + time +  " VelocityX: " + velocityVector.getI() + " VelocityY: " + velocityVector.getJ() + " ThetaChange: " + thetaChange + " CurentX: " + currentX + " CurrentY: " + currentY);
            // System.out.println("Inside Last Point" + " CurrentPointTime: " + currentPointTime + " Time: " + time + " VelocityX: " + velocityVector.getI() + " VelocityY: " + velocityVector.getJ() + "Theta: " + currentTheta + " ThetaChange: " + thetaChange + " CurentX: " + currentX + " CurrentY: " + currentY);

            return true;
        }

        nextPoint = pathPointsJSON.getJSONObject(currentPointIndex + 1);
        previousPoint = pathPointsJSON.getJSONObject(currentPointIndex - 1);

        double nextPointTime = nextPoint.getDouble("time");
        double previousPointTime = previousPoint.getDouble("time");

        double timeDiffT1 = (currentPointTime - previousPointTime);
        // double timeTonextPoint = (nextPointTime - time);
        double t1 = (timeDiffT1 * turnTimePercent) + previousPointTime;

        double timeDiffT2 = nextPointTime - currentPointTime;
        double t2 = (timeDiffT2 * (1 - turnTimePercent)) + currentPointTime;

        if(time < t1) {
            if(Math.abs((currentPointTime - time)) < cyclePeriod) {
                // System.out.println("WITHIN CYCLE PERIOD");
                velocityX = currentXVelocity;
                velocityY = currentYVelocity;
                thetaChange = currentThetaVelocity;
            }
            else {
                // System.out.println("NOT WITHIN CYCLE PERIOD");
                velocityX = (currentPoint.getDouble("x") - currentX)/(currentPointTime - time);
                velocityY = (currentPoint.getDouble("y") - currentY)/(currentPointTime - time);
                thetaChange = (currentPoint.getDouble("angle") - currentTheta)/(currentPointTime - time);
            }
        }
        else if(time >= t1 && time < t2) {
            double t1X = (currentPoint.getDouble("x") - previousPoint.getDouble("x"))/timeDiffT1;
            double t1Y = (currentPoint.getDouble("y") - previousPoint.getDouble("y"))/timeDiffT1;
            double t1Theta = (currentPoint.getDouble("angle") - previousPoint.getDouble("angle"))/timeDiffT1;

            double t2X = (nextPoint.getDouble("x") - currentPoint.getDouble("x"))/timeDiffT2;
            double t2Y = (nextPoint.getDouble("y") - currentPoint.getDouble("y"))/timeDiffT2;
            double t2Theta = (nextPoint.getDouble("angle") - currentPoint.getDouble("angle"))/timeDiffT2;

            double idealAccelX = (t2X - t1X)/(t2 - t1);
            double idealAccelY = (t2Y - t1Y)/(t2- t1);
            double idealAccelTheta = (t2Theta - t1Theta)/(t2 - t1);

            double t1XPosition = (t1X * t1) + previousPoint.getDouble("x");
            double t1YPosition = (t1Y * t1) + previousPoint.getDouble("y");
            double t1ThetaPosition = (t1Theta * t1) + previousPoint.getDouble("angle");

            double interpTime = time - t1 + cyclePeriod;

            double idealPositionX = (idealAccelX/2) * (Math.pow(interpTime, 2)) + t1X * (interpTime) + t1XPosition;
            double idealPositionY = (idealAccelY/2) * (Math.pow(interpTime, 2)) + t1Y * (interpTime) + t1YPosition;
            double idealPositionTheta = (idealAccelTheta/2) * (Math.pow(interpTime, 2)) + t1Theta * (interpTime) + t1ThetaPosition;

            double idealVelocityX = t1X + (idealAccelX * interpTime);
            double idealVelocityY = t1Y + (idealAccelY * interpTime);
            double idealVelocityTheta = t1Theta + (idealAccelTheta * interpTime);

            velocityX = (idealPositionX - currentX) * interpolationCorrection + idealVelocityX;
            velocityY = (idealPositionY - currentY) * interpolationCorrection + idealVelocityY;
            thetaChange = (idealPositionTheta - currentTheta) * interpolationCorrection + idealVelocityTheta;

            // System.out.println(Math.round(time * (100.0))/100.0 + ", " + Math.round(velocityX * 100.0)/100.0 + ", " + Math.round(currentXVelocity * 100.0)/100.0);
            // System.out.println("INSIDE INTERPOLATION" + " Time: " + time + " Point: " + currentPointIndex + " X: " + currentX + " CurrentXVelocity: " + currentXVelocity + " Y: " + currentY +  " AccelX: " + accelX + " AccelY: " + accelY);
        }
        else if(time >= t2) {
            if(Math.abs((nextPointTime - time)) < cyclePeriod) {
                velocityX = currentXVelocity;
                velocityY = currentYVelocity;
                thetaChange = currentThetaVelocity;
            }
            else {
                velocityX = (nextPoint.getDouble("x") - currentX)/(nextPointTime - time);
                velocityY = (nextPoint.getDouble("y") - currentY)/(nextPointTime - time);
                thetaChange = (nextPoint.getDouble("angle") - currentTheta)/(nextPointTime - time);
            }
        }

        Vector velocityVector = new Vector(velocityX, velocityY);

        System.out.println("Time: " + time +  " VelocityX: " + velocityVector.getI() + " VelocityY: " + velocityVector.getJ() + " ThetaChange: " + thetaChange + " CurentX: " + currentX + " CurrentY: " + currentY);

        autoDrive(velocityVector, thetaChange); 
        // velocityArray[0] = velocityX;
        // velocityArray[1] = velocityY;
        // velocityArray[2] = thetaChange;
        // return velocityArray;         
        return true;
    }

    private Command DriveDefault() {
        return null;
    }
 
    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}