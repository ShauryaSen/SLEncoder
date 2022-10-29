package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.Ports.Shooter.*;
import static com.stuypulse.robot.constants.Settings.Shooter.*;
import static com.stuypulse.robot.constants.Motors.Shooter.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.util.SLEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter extends SubsystemBase {

    private SmartNumber targetRPM;

    private CANSparkMax shooterMotor;
    private CANSparkMax shooterFollower;

    // rev encoders
    private RelativeEncoder shooterMotorEncoder;
    private RelativeEncoder shooterFollowerEncoder;

    // Stuylib encoders
    private SLEncoder slShooterFollowerEncoder;
    private SLEncoder slShooterMotorEncoder;
    private SLEncoder slFeederMotorEncoder;

    private Controller shooterController;
    private Controller feederController;

    private CANSparkMax feederMotor;
    private RelativeEncoder feederMotorEncoder;

    public Shooter(SLEncoder slEncoder) {
        slShooterFollowerEncoder = slEncoder;
        slShooterMotorEncoder = slEncoder; 
        slFeederMotorEncoder = slEncoder;
        setup();
    }

    public Shooter() {
        shooterMotorEncoder = shooterMotor.getEncoder();
        shooterFollowerEncoder = shooterFollower.getEncoder();
        feederMotorEncoder = feederMotor.getEncoder();
        setup();
    }

    private void setup() {
        targetRPM = new SmartNumber("Shooter/TargetRPM", 0.0);

        shooterMotor = new CANSparkMax(SHOOTER_MOTOR, MotorType.kBrushless);
        ShooterMotorConfig.configure(shooterMotor);
        
        shooterFollower = new CANSparkMax(SHOOTER_FOLLOWER, MotorType.kBrushless);
        ShooterFollowerConfig.configure(shooterFollower);

        shooterController = new PIDController(ShooterPID.kP, ShooterPID.kI, ShooterPID.kD)
            .add(new Feedforward.Flywheel(ShooterFF.kS, ShooterFF.kV, ShooterFF.kA ).velocity());
        feederController = new PIDController(FeederPID.kP, FeederPID.kI, FeederPID.kD)
            .add(new Feedforward.Flywheel(FeederFF.kS, FeederFF.kV, FeederFF.kA).velocity());

        feederMotor = new CANSparkMax(FEEDER_MOTOR, MotorType.kBrushless);
        FeederMotorConfig.configure(feederMotor);
    }

    public void setTargetRPM(double RPM){
        targetRPM.set(RPM);
    }

    public double getShooterRPM() {
        double shooterRPM;
        if (shooterMotorEncoder != null) {
            shooterRPM = (shooterMotorEncoder.getVelocity() + shooterFollowerEncoder.getVelocity()) / 2;
        } else {
            shooterRPM = (slShooterMotorEncoder.getVelocity() + slShooterFollowerEncoder.getVelocity()) / 2;
        }

        return shooterRPM;
    }

    public double getFeederRPM() {
        double feederRPM;
        if (shooterMotorEncoder != null) {
            feederRPM = feederMotorEncoder.getVelocity();
        } else {
            feederRPM = slFeederMotorEncoder.getVelocity();
        }
        return feederRPM;
    }

    private double getShooterVoltage() {
        return shooterController.update(targetRPM.get(), getShooterRPM());
    }

    private double getFeederVoltage() {
        final double feederTargetRPM = targetRPM.get() * FeederFF.FEEDER_RPM_MULTIPLIER.get();
        return feederController.update(feederTargetRPM, getFeederRPM());   
    }
    
    @Override
    public void periodic() {
        final double shooterVoltage = getShooterVoltage();
        final double feederVoltage = getFeederVoltage();

        shooterMotor.setVoltage(shooterVoltage);
        shooterFollower.setVoltage(shooterVoltage);

        feederMotor.setVoltage(feederVoltage);

        SmartDashboard.putNumber("Shooter/Shooter Voltage", shooterVoltage);
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederVoltage);

        SmartDashboard.putNumber("Shooter/Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter/Feeder RPM", getFeederRPM());
    }
}