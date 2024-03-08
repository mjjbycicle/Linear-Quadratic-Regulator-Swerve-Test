package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotInfo.SwerveInfo;
import frc.robot.Constants.SwerveModuleConfig;
import math.MathUtil;

public class SwerveModule extends SubsystemBase{
    private final TalonFX driver, rotator;
    private final CANcoder rotationEncoder;
    private final LQR rotationController;
    private final LQR driverController;

    private final double rotationOffsetDegrees;
    private final double driveConversionFactor;
    private double targetSpeed = 0;

    public SwerveModule(SwerveModuleConfig swerveModuleData) {
        driveConversionFactor = (1. / 2048) * (1 / 6.55) * (0.1016) * Math.PI;

        driver = switch (Constants.currentRobot) {
            case ZEUS -> new TalonFX(swerveModuleData.driveMotorID());
            case CADENZA -> new TalonFX(swerveModuleData.driveMotorID(), "Canivore1");
        };
        rotator = switch (Constants.currentRobot) {
            case ZEUS -> new TalonFX(swerveModuleData.rotatorMotorID());
            case CADENZA -> new TalonFX(swerveModuleData.rotatorMotorID(), "Canivore1");
        };
        rotator.setInverted(true);

        driver.setNeutralMode(NeutralModeValue.Brake);
        rotator.setNeutralMode(NeutralModeValue.Brake);

        rotationEncoder = switch(Constants.currentRobot) {
            case ZEUS -> new CANcoder(swerveModuleData.encoderID());
            case CADENZA -> new CANcoder(swerveModuleData.encoderID(), "Canivore1");
        };

        rotationOffsetDegrees = swerveModuleData.rotationOffset();

        rotationController = SwerveInfo.SWERVE_ROTATOR_LQR.create();

        driverController = SwerveInfo.SWERVE_DRIVER_LQR;
    }

    private static boolean isNegligible(SwerveModuleState state) {
        return state.speedMetersPerSecond < 0.0001;
    }

    public void reset() {
        rotationController.reset();
    }

    public void periodic() {
        int moduleID = rotator.getDeviceID() / 2;
        double currModuleRotation = getRotationInDegrees();
        rotationController.setSetpoint(currModuleRotation, currModuleRotation);
        driverController.setSetpoint(targetSpeed, getDriveVelocity());

        SmartDashboard.putString("Module %d current rotation".formatted(moduleID), "%.2f degrees".formatted(currModuleRotation));

        double rotatorOutput = rotationController.getVoltage();
        double driverOutput = - driverController.getVoltage(); //drive change

        SmartDashboard.putString("Module %d target speed".formatted(moduleID), "%.2f".formatted(targetSpeed));
        SmartDashboard.putString("Module %d target rotation".formatted(moduleID), "%.2f degrees".formatted(rotationController.getSetpoint()));
        SmartDashboard.putString("Module %d rotator PID output".formatted(moduleID), "%.2f".formatted(rotatorOutput));

        rotator.set(rotatorOutput);
        driver.set(driverOutput * SwerveInfo.MOVEMENT_SPEED); //drive change-to do: change targetSpeed to driverPIDOutput
    }

    public double getRotationInDegrees() {
        double rawRotationInDegrees = rotationEncoder.getAbsolutePosition().getValue() * 360 - rotationOffsetDegrees;
        return MathUtil.mod(rawRotationInDegrees, -180, 180);
    }

    private void setTargetSpeed(double targetSpeed) {
        this.targetSpeed = targetSpeed;
    }

    private void setRotation(double degrees) {
        rotationController.setSetpoint(degrees, rotationEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public double getDriveVelocity() {
        return driver.getVelocity().getValue() * driveConversionFactor;
    }

    public double getDriveDistance() {
        return driver.getPosition().getValue() * driveConversionFactor;
    }

    public double getTurningVelocity() {
        return rotationEncoder.getVelocity().getValue();
    }

    public double getTurningPosition() {
        double rawRotation = rotationEncoder.getAbsolutePosition().getValue() - rotationOffsetDegrees;

        return MathUtil.mod(rawRotation, -180, 180);
    }

    public SwerveModuleState getState() {
        double moduleSpeed = getDriveVelocity();
        double rotationDegrees = getRotationInDegrees();
        Rotation2d rotation = Rotation2d.fromDegrees(rotationDegrees);
        return new SwerveModuleState(moduleSpeed, rotation);
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getRotationInDegrees()));
        setTargetSpeed(optimizedState.speedMetersPerSecond);
        setRotation(optimizedState.angle.getDegrees());
    }

    public void stop() {
        setTargetSpeed(0);
        setRotation(getRotationInDegrees());

        driver.stopMotor();
        rotator.stopMotor();
    }

    public void setBrakeMode(){
        driver.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setCoastMode(){
        driver.setNeutralMode(NeutralModeValue.Coast);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getTurningPosition()));
    }
}