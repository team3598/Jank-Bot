package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;


public class PoseSubsystem extends SubsystemBase {
    public static double timeMS = Timer.getFPGATimestamp();
    final Field2d field = new Field2d();

    private final CommandSwerveDrivetrain drivetrain;
        private final List<String> limelightNames = List.of("limelight-bright");//, "limelight-fright"); //include limelight-fright later on when you figure out how to get avgs between two limelights
        private int loopCounter = 0;

    
    public PoseSubsystem(CommandSwerveDrivetrain drivetrain) { //constructor
        this.drivetrain = drivetrain;        
        SmartDashboard.putData(field); 
    }
    
    private Vector<N3> calculateStdDev(double distance) {
        double xyUncertainty = Math.max(0.1, 0.1 * Math.pow(distance, 4)); //there is Math.max because it'll spaz out if we dont give it a floor...
        double thetaUncertainty = Math.max(0.4, 0.4 * Math.pow(distance, 4));
        Vector<N3> calculatedStdDevs = VecBuilder.fill(xyUncertainty, xyUncertainty, thetaUncertainty); 
        return calculatedStdDevs;
    }

    private void updateVision(String name) {
        if (!LimelightHelpers.getTV(name)) {return;} //if limelight not exist, then dont even bother running the rest
    
        var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            
        if (mt2 == null || mt2.tagCount == 0 || mt2.avgTagDist > 4.0 || 
            mt1 == null || mt1.tagCount == 0 || mt1.avgTagDist > 4.0) 
            {return;}

        LimelightHelpers.SetRobotOrientation(name, mt1.pose.getRotation().getDegrees(), 0, 0, 0, 0, 0); //gives MT2 the current rotation of the bot
    
        Vector<N3> stdDevs = calculateStdDev(mt1.avgTagDist); //we need our std dev for rotation to be high so that the robot uses the gyro for drivetrain, but the stddev will nudge the gyro in the right direction (if it drifts)
    
        drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);
    }
    
    public Pose2d getCurrentPose() {
        return drivetrain.getState().Pose;
    }

    public double getDistToTarget(Translation2d targetPosition) {
        return drivetrain.getState().Pose.getTranslation().getDistance(targetPosition);
    }

    public void printCurrentPose(){
        System.out.println(drivetrain.getState().Pose);
    }
    
    @Override
    public void periodic() {
        timeMS = Timer.getFPGATimestamp(); //20 ms
        String activeCamera = limelightNames.get(loopCounter % limelightNames.size()); //i do this because it is really taxing on the RoboRio to calculate everything and process two limelights at the same time. by doing this, it's switching back and forth between the two which should give me some less lag
        loopCounter++;
        updateVision(activeCamera);
        //printCurrentPose();
        field.setRobotPose(drivetrain.getState().Pose);
    }
}
