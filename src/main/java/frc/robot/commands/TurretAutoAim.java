package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class TurretAutoAim extends Command {
    private final TurretSubsystem Turret;
    private final PoseSubsystem Pose;
    private final Translation2d Target;

    public TurretAutoAim(TurretSubsystem turret, PoseSubsystem pose) {
        Turret = turret;
        Pose = pose;
        Target = new Translation2d(4.625, 4.035); //hub position
        
        addRequirements(Turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = Pose.getCurrentPose();
        
        double dx = Target.getX() - robotPose.getX();
        double dy = Target.getY() - robotPose.getY();

        Rotation2d angleToTarget = new Rotation2d(dx, dy);
        
        Rotation2d targetAngleRelative = angleToTarget.minus(robotPose.getRotation());
        
        Turret.goToAngle(targetAngleRelative.getDegrees());
        System.out.println(targetAngleRelative.getDegrees());

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}