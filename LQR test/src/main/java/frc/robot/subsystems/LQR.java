package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LQR extends SubsystemBase {
    private final LinearSystemLoop<N1, N1, N1> loop;
    private double setPoint;
    private double kV, kA, modelAccuracy, encoderAccuracy, kQ, kR, frameTime, maxVoltage;
    public LQR(double kV,
               double kA,
               double modelAccuracy,
               double encoderAccuracy,
               double kQ,
               double kR,
               double frameTime,
               double maxVoltage){
        LinearSystem<N1, N1, N1> swervePlant = LinearSystemId.identifyVelocitySystem(kV, kA);
        KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
                Nat.N1(),
                Nat.N1(),
                swervePlant,
                VecBuilder.fill(modelAccuracy),
                VecBuilder.fill(encoderAccuracy),
                frameTime);
        LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(
                swervePlant,
                VecBuilder.fill(kQ),
                VecBuilder.fill(kR),
                frameTime);
        loop = new LinearSystemLoop<>(swervePlant, controller, observer, maxVoltage, frameTime);
        this.kV = kV; this.kA = kA; this.modelAccuracy = modelAccuracy; this.encoderAccuracy = encoderAccuracy;
        this.kQ = kQ; this.kR = kR; this.frameTime = frameTime; this.maxVoltage = maxVoltage;
    }
    /*
    requires
     */
    public void setSetpoint(double setPoint, double encoderMeasurement){
        loop.setNextR(setPoint);
        this.setPoint = setPoint;
        correct(encoderMeasurement);
    }

    public void correct(double encoderMeasurment){
        loop.correct(VecBuilder.fill(encoderMeasurment));
    }

    public double getVoltage(){
        return loop.getU(0);
    }

    public void reset(){
        loop.setNextR(0);
        this.setPoint = 0;
        correct(0);
    }

    public LQR create() {
        return new LQR(kV, kA, modelAccuracy, encoderAccuracy, kQ, kR, frameTime, maxVoltage);
    }

    public double getSetpoint(){
        return setPoint;
    }
}
