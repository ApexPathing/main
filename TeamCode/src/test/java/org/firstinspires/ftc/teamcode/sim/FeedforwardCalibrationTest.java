package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.junit.Test;
import static org.junit.Assert.assertTrue;
import java.util.Locale;

/** Bench identification: fit holding power first, then acceleration, without feedback. */
public class FeedforwardCalibrationTest {
    private static final double RADIUS = 1.889765;
    private static double acceleration(SimMotor motor, double speed, double power) {
        motor.setRollVelocity(speed / RADIUS);
        motor.setPower(power);
        motor.update(0.001);
        return motor.getAcceleration() * RADIUS;
    }
    private static double powerFor(SimMotor motor, double speed, double accel) {
        double lo=0, hi=1;
        for(int i=0;i<40;i++) {
            double mid=(lo+hi)/2;
            if(acceleration(motor,speed,mid)>accel) hi=mid; else lo=mid;
        }
        return (lo+hi)/2;
    }
    @Test public void characterizeThenValidateOnIndependentVelocities() {
        SimMotor motor=(SimMotor)ApexSimulation.createHardware().hardwareMap.get(
                DcMotorEx.class,ApexSimulation.FRONT_LEFT_MOTOR);
        double n=0,sx=0,sy=0,sxx=0,sxy=0;
        for(double v=6;v<=42;v+=6) {
            double u=powerFor(motor,v,0);
            n++;sx+=v;sy+=u;sxx+=v*v;sxy+=v*u;
        }
        double kv=(n*sxy-sx*sy)/(n*sxx-sx*sx);
        double ks=(sy-kv*sx)/n;
        double numerator=0,denominator=0;
        for(double v=6;v<=42;v+=6) for(double a:new double[]{15,30,45}) {
            numerator+=a*(powerFor(motor,v,a)-ks-kv*v);
            denominator+=a*a;
        }
        double ka=numerator/denominator;
        System.out.printf(Locale.US,"CANDIDATE ks=%.12f kv=%.12f ka=%.12f%n",ks,kv,ka);
        double squared=0;int count=0;
        for(double v=9;v<=39;v+=6) {
            double power=ks+kv*v;
            motor.setRollVelocity(v/RADIUS);
            motor.setPower(power);
            for(int i=0;i<1000;i++) motor.update(.001);
            double actual=motor.getVelocity()*RADIUS;
            squared+=(actual-v)*(actual-v);count++;
            System.out.printf(Locale.US,"HOLD target=%.1f actualAfter1s=%.3f%n",v,actual);
        }
        double rmse=Math.sqrt(squared/count);
        System.out.printf(Locale.US,"HOLD RMSE %.3f%n",rmse);
        assertTrue("Affine model must hold independent speeds within 2 in/s RMS",rmse<2);
        squared=0; count=0;
        motor.setRollVelocity(6/RADIUS);
        for(int i=0;i<50;i++) {
            double target=6+30*i*.02;
            double error=motor.getVelocity()*RADIUS-target;
            squared+=error*error; count++;
            motor.setPower(ks+kv*target+ka*30);
            for(int k=0;k<20;k++)motor.update(.001);
        }
        double accelerationRmse=Math.sqrt(squared/count);
        System.out.printf(Locale.US,"ACCELERATION feedforward-only RMSE %.3f%n",accelerationRmse);
        assertTrue("Acceleration fit must track an independent ramp",accelerationRmse<2.5);
        for(double gain:new double[]{0,.01,.02,.04,.06,.08,.10}) {
            squared=0;count=0; double peak=0;
            motor.setRollVelocity(6/RADIUS);
            for(int i=0;i<200;i++) {
                double t=i*.02;
                double target=t<1?6+30*t:t<2?36:t<3?36-30*(t-2):6;
                double a=t<1?30:0; // Braking is deliberately feedback-owned.
                double measured=motor.getVelocity()*RADIUS;
                double power=ks+kv*target+ka*a+gain*(target-measured);
                motor.setPower(power);
                for(int k=0;k<20;k++)motor.update(.001);
                double error=measured-target;
                squared+=error*error;count++;peak=Math.max(peak,error);
            }
            System.out.printf(Locale.US,"FEEDBACK gain=%.3f RMSE=%.3f peakOverspeed=%.3f%n",
                    gain,Math.sqrt(squared/count),peak);
        }
    }
}
