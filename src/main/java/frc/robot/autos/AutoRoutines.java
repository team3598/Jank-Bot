package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine T1OPRoboticsAuto() {
        final AutoRoutine routine = m_factory.newRoutine("T1Robotics Auto");
        final AutoTrajectory autoTrajectory = routine.trajectory("T1OPRoboticsAuto");

        routine.active().onTrue(
            autoTrajectory.resetOdometry()
                .andThen(autoTrajectory.cmd())
        );
        return routine;
    }

    public AutoRoutine T2OPRoboticsAuto() {
        final AutoRoutine routine = m_factory.newRoutine("T2Robotics Auto");
        final AutoTrajectory autoTrajectory = routine.trajectory("T2OPRoboticsAuto");

        routine.active().onTrue(
            autoTrajectory.resetOdometry()
                .andThen(autoTrajectory.cmd())
        );
        return routine;
    }

    public AutoRoutine shootingTest() {
        final AutoRoutine routine = m_factory.newRoutine("ShootingTest Auto");
        final AutoTrajectory autoTrajectory = routine.trajectory("ShootingTest");

        routine.active().onTrue(
            autoTrajectory.resetOdometry()
                .andThen(autoTrajectory.cmd())
        );
        return routine;
    }

    public AutoRoutine T2BeachBoys() {
        final AutoRoutine routine = m_factory.newRoutine("T2BeachBoys Auto");
        final AutoTrajectory autoTrajectory = routine.trajectory("T2BeachBoys");

        routine.active().onTrue(
            autoTrajectory.resetOdometry()
                .andThen(autoTrajectory.cmd())
        );
        return routine;
    }

    /*public AutoRoutine OPRoboticsAuto() {
        final AutoRoutine routine = m_factory.newRoutine("OP Robotics Auto");
        
    }*/
}
