package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.tools.math.Vector;

public class SwerveModule extends SubsystemBase {
    private final double mZeroOffset;
    
    private double moduleNum = 0;

	private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    
    private double turnVectorX = 0;
    private double turnVectorY = 0;

    private double turnPower = 0.75;

    private Vector turnVector = new Vector(0, 0);

    private final CANCoder absoluteEncoder;

    public SwerveModule(int moduleNumber, TalonFX mAngleMotor, TalonFX mDriveMotor, double zeroOffset, CANCoder mAbsoluteEncoder) {
        // sets up the module by defining angle motor and drive motor
        angleMotor = mAngleMotor;
        driveMotor = mDriveMotor;

        mZeroOffset = zeroOffset;
        moduleNum = moduleNumber;

        // defines absolute encoder
        absoluteEncoder = mAbsoluteEncoder;

        // configures angle motor PID, output, etc.
        angleMotor.configPeakOutputForward(1);
        angleMotor.configPeakOutputReverse(-1);
        angleMotor.configVoltageCompSaturation(11.7);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.setSensorPhase(true);
        angleMotor.selectProfileSlot(0, 0);
        angleMotor.config_kF(0, 0.0);
        angleMotor.config_kP(0, 0.5);
        angleMotor.config_kI(0, 0);
        angleMotor.config_kD(0, 0);

        // sets drive motor to brake
        driveMotor.setNeutralMode(NeutralMode.Brake);

        // this block creates the turn vector based on the module's relation to the robot(stays constant because robot is square)
        if(moduleNumber == 1) {
            turnVector.setI(Math.sqrt(2)/2.0);
            turnVector.setJ(Math.sqrt(2)/2.0);
        }
        if(moduleNumber == 2) {
            turnVector.setI(-Math.sqrt(2)/2.0);
            turnVector.setJ(Math.sqrt(2)/2.0);
        }
        if(moduleNumber == 3) {
            turnVector.setI(-Math.sqrt(2)/2.0);
            turnVector.setJ(-Math.sqrt(2)/2.0);
        }
        if(moduleNumber == 4) {
            turnVector.setI(Math.sqrt(2)/2.0);
            turnVector.setJ(-Math.sqrt(2)/2.0);
        }
    }

    // this method returns the angle of the joystick given the X and Y components
    public double getJoystickAngle(double joystickUp, double joystickSide) {
        double joystickAngle = Math.atan2(-joystickUp, joystickSide);
        return joystickAngle;
    }

    // this method sets the angle motor to move to a specific angle(in radians) and then sets the drive motors to the motor percent
    public void setAnglePID(double targetAngle, double velocity){
        angleMotor.set(ControlMode.Position, (radiansToTics((targetAngle))));
        setDriveMotorVelocity(velocity);
    }

    // velocity in tics/100 milliseconds
    public void setDriveMotorVelocity(double velocity) {
        driveMotor.set(ControlMode.Velocity, velocity);
    }

    // convert from radians to ticks
    public double radiansToTics(double radians) {
        double outputTics = 0.5 * radians * (Constants.FALCON_TICS_PER_ROTATION * Constants.STEER_GEAR_RATIO/Math.PI);
        return outputTics;
    }

    // convert from ticks to radians
    public double ticsToRadians(double tics) {
        double outputRadians = 2 * Math.PI * (tics/(Constants.FALCON_TICS_PER_ROTATION * Constants.STEER_GEAR_RATIO));
        return outputRadians;
    }

    // convert from radians to degrees
    public double radiansToDegrees(double radians) {
        double outputDegrees = 180 * radians/Math.PI;
        return outputDegrees;
    }

    // convert from degrees to radians
    public double degreesToRadians(double degrees) {
        double outputRadians = Math.PI * degrees/180;
        return outputRadians;
    }

    // sets the drive motor to the specified percent
    public void setDriveMotors(double percent) {
        driveMotor.set(ControlMode.PercentOutput, percent);
    }

    public void postDriveMotorTics() {
        // System.out.println("Tics: " + driveMotor.getSelectedSensorPosition());
    }

    public SwerveModuleState getState(double navxOffset) {
        return new SwerveModuleState((ticsPer100MSToSpeed(driveMotor.getSelectedSensorVelocity())), new Rotation2d(Math.toRadians(getAbsolutePosition() - navxOffset)));
    }

    // method run when robot boots up, sets up Current Limits, as well as tells internal encoder where it actually is
    public void init() {
        angleMotor.setSelectedSensorPosition(radiansToTics(degreesToRadians(absoluteEncoder.getAbsolutePosition())));
        angleMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0 ,0));

        angleMotor.config_kP(0, 0.1);
        angleMotor.config_kI(0, 0.0);
        angleMotor.config_kD(0, 0.1);
        
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0 , 0));
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.setInverted(true);;

        driveMotor.config_kP(0, 0.0849998);
        driveMotor.config_kI(0, 0.0008997917);
        driveMotor.config_kD(0, 0.899999857);
    }

    // returns angle motor position in ticks
    public double getAngleMotorEncoder(){
        return angleMotor.getSelectedSensorPosition();
    }

    // returns Cancoder position in degrees
    public double getAbsolutePosition() {
        return absoluteEncoder.getAbsolutePosition();
    }

    // returns modules position in radians
    public double getModulePosition(){
        return ticsToRadians(getAngleMotorEncoder());
    }

    // input is speed in meters/second
    public double speedToTicsPerSecond(double speed) {
        double tics = (speed * Constants.GEAR_RATIO * Constants.FALCON_TICS_PER_ROTATION)/(Constants.WHEEL_CIRCUMFRENCE);
        // System.out.println(tics);
        return tics;
    }

    public double speedToTicsPer100MS(double speed) {
        double ticsPerSecond = speedToTicsPerSecond(speed);
        double ticsPer100MS = ticsPerSecond/10;
        return ticsPer100MS;
    }

    public double ticsPerSecondToSpeed(double tics) {
        double speed = (tics * Constants.WHEEL_CIRCUMFRENCE)/(Constants.GEAR_RATIO * Constants.FALCON_TICS_PER_ROTATION);
        return speed;
    }

    public double ticsPer100MSToSpeed(double tics) {
        double speedPer100MS = ticsPerSecondToSpeed(tics);
        double speed = speedPer100MS * 10;
        return speed;
    }

    // method to determine the angle of each wheel and the percent to set each module to
    // speedVector is a <x, y> ground speed relative to the field measured in meters/second, turnRate is in radians/second relatie to the robot(positive is counterclockwise), navxOffset is in radians(positive is counterclockwise)
    public void velocityDrive(Vector speedVector, double turnRate, double navxOffset) {

        if(Math.abs(speedVector.getI()) < 0.0001 && Math.abs(speedVector.getJ()) < 0.0001 && Math.abs(turnRate) < 0.01) {
            driveMotor.set(ControlMode.Velocity, 0.0);
            // angleMotor.set(ControlMode.PercentOutput, 0);
        }
        else {
            // finds the target angle based on controller XY and then factors in navx offset
            double targetAngle = Math.atan2(speedVector.getJ(), speedVector.getI());
            targetAngle = targetAngle + degreesToRadians(navxOffset);

            // converts target angle from (r, theta) to <x, y>
            double hypotenuse = Math.sqrt(Math.pow(speedVector.getI(), 2) + Math.pow(speedVector.getJ(), 2));
            double navxAdjustedX = hypotenuse * Math.cos(targetAngle);
            double navxAdjustedY = hypotenuse * Math.sin(targetAngle);

            // factoring in turn percent to turn vector constants so that it doesn't turn too much
            double turnRateMetersPerSec = turnRate * Constants.ROBOT_RADIUS;
            double turnX = turnRateMetersPerSec * turnVector.getI();
            double turnY = turnRateMetersPerSec * turnVector.getJ();

            // add the turn vector plus the strafe vector together
            Vector adjustedVector = new Vector(turnX + navxAdjustedX, turnY + navxAdjustedY);

            // compute adjusted angle and power based on adjusted vector
            double motorFieldSpeed = Math.sqrt(Math.pow(adjustedVector.getI(), 2) + Math.pow(adjustedVector.getJ(), 2));

            if(motorFieldSpeed > Constants.TOP_SPEED) {
                System.out.println("Module Number: " + moduleNum + " cannot reach requested speed of " + motorFieldSpeed);
                motorFieldSpeed = Constants.TOP_SPEED;
            }

            double adjustedAngle = Math.atan2(adjustedVector.getJ(), adjustedVector.getI());

            double velocityTicsPer100MS = (speedToTicsPer100MS(motorFieldSpeed));

            // find initial angle of module to use optimizer
            double initAngle = getModulePosition();
            double boundedInitAngle = initAngle%Math.toRadians(360);

            double caseOneAngle = adjustedAngle;
            if(adjustedAngle > boundedInitAngle){
                caseOneAngle = adjustedAngle - Math.toRadians(360);
            }
            //Case one moves clockwise
            double caseTwoAngle = adjustedAngle;
            if(adjustedAngle < boundedInitAngle){
            caseTwoAngle = adjustedAngle + Math.toRadians(360);

            }
            //Case two moves counterclockwise
            double caseThreeAngle = adjustedAngle + Math.toRadians(180);
            double caseFourAngle = (adjustedAngle + Math.toRadians(180)) - Math.toRadians(360);

            // check distance to each case
            double distanceOne = Math.abs(boundedInitAngle - caseOneAngle);
            double distanceTwo = Math.abs(boundedInitAngle - caseTwoAngle);
            double distanceThree = Math.abs(boundedInitAngle - caseThreeAngle);
            double distanceFour = Math.abs(boundedInitAngle - caseFourAngle);

            // based on which distance is smallest, setAnglePID with +- motor percent and angle
            if(motorFieldSpeed > 0.01){
                if((distanceOne < distanceTwo) && (distanceOne < distanceThree) && (distanceOne < distanceFour)){
                    setAnglePID((caseOneAngle - boundedInitAngle + initAngle), velocityTicsPer100MS);
                }
                if((distanceTwo < distanceOne) && (distanceTwo < distanceThree) && (distanceTwo < distanceFour)){
                    setAnglePID((caseTwoAngle - boundedInitAngle + initAngle), velocityTicsPer100MS);
                }
                if((distanceThree < distanceOne) && (distanceThree < distanceTwo) && (distanceThree < distanceFour)){
                    setAnglePID((caseThreeAngle - boundedInitAngle + initAngle),  -velocityTicsPer100MS);
                }
                if((distanceFour < distanceOne) && (distanceFour < distanceTwo) && (distanceFour < distanceThree)){
                    setAnglePID((caseFourAngle - boundedInitAngle + initAngle),  -velocityTicsPer100MS);
                }
            }
            else{
                driveMotor.set(ControlMode.PercentOutput, 0);
            }
        }

        
    }  

    public void testDrive() {
        // if(OI.driverController.getAButton()) {
        //     setAnglePID((Math.PI)/2, 10000);
        // }
        // if(OI.driverController.getBButton()) {
        //     setAnglePID((Math.PI), 10000);
        // }
        // if(OI.driverController.getYButton()) {
        //     setAnglePID((3 * Math.PI)/2, 10000);
        // }

        driveMotor.set(ControlMode.PercentOutput, 1);

        // driveMotor.set(ControlMode.Ve)
    }

}
