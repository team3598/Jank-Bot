package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

public class SimpleCRT {
    private final double ratioA;
    private final double ratioB; 
    private final double minMechRotations;
    private final double maxMechRotations;

    /**
     * @param teethA Teeth on Encoder A Gear (e.g. 10)
     * @param teethB Teeth on Encoder B Gear (e.g. 11)
     * @param ringTeeth Teeth on Turret Ring (e.g. 100)
     * @param minDegrees Minimum physical angle (e.g. -20)
     * @param maxDegrees Maximum physical angle (e.g. 275)
     */
    public SimpleCRT(double teethA, double teethB, double ringTeeth, double minDegrees, double maxDegrees) {
        this.ratioA = ringTeeth / teethA; 
        this.ratioB = ringTeeth / teethB;
        
        this.minMechRotations = minDegrees / 360.0;
        this.maxMechRotations = maxDegrees / 360.0;
    }

    public double getTrueAngle(double rawPosA, double rawPosB) {
        double posA = MathUtil.inputModulus(rawPosA, 0.0, 1.0);
        double posB = MathUtil.inputModulus(rawPosB, 0.0, 1.0);

        double bestAngle = Double.NaN;
        double minError = Double.MAX_VALUE;

        int minK = (int) Math.floor(minMechRotations * ratioA) - 1;
        int maxK = (int) Math.ceil(maxMechRotations * ratioA) + 1;

        for (int k = minK; k <= maxK; k++) {
            double totalRotationsA = k + posA;
            
            double candidateMechRot = totalRotationsA / ratioA;

            if (candidateMechRot < minMechRotations || candidateMechRot > maxMechRotations) {
                continue; 
            }

            double expectedRotationsB = candidateMechRot * ratioB;
            double expectedPosB = MathUtil.inputModulus(expectedRotationsB, 0.0, 1.0);

            double error = Math.abs(expectedPosB - posB);
            if (error > 0.5) error = 1.0 - error;

            if (error < 0.05) { 
                if (error < minError) {
                    minError = error;
                    bestAngle = candidateMechRot * 360.0;
                }
            }
        }

        return bestAngle;
    }
}