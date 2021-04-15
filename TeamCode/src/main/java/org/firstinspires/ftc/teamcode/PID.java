package org.firstinspires.ftc.teamcode;
import android.sax.StartElementListener;

import java.util.ArrayList;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double kp, ki, kd;
    private double totalError, lastError, lastTime;
    private ArrayList<Double> errors;

    public PID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        lastTime = 0;
        errors = new ArrayList<Double>();
    }

    public double getP(double error) {
        return kp * error;
    }

    public double getI(double error) {
        if (Math.abs(error) < 1)
            totalError = 0;
        return ki * totalError;
    }

    public double getD(double currentError, ElapsedTime runtime) {
        return kd * (currentError - lastError)/(runtime.time()-lastTime);
    }

    public boolean checkCorrection(double error){

        if(Math.abs(errors.get(errors.size()-1)) < Math.abs(error))
            return true;
        else
            return false;

    }

    public double getCorrection(double error, ElapsedTime runtime) {
       /* if(errors.size() != 0) {
            totalError += error;
            double potentialError = getP(error) + getI(error) + getD(error, runtime);
            if (Math.abs(error) < 0.001 || (checkCorrection(potentialError))) {
                totalError -= error;
                return 0;
            }
            totalError -= error;


        }*/

        if (Math.abs(error) < 0.001) {
            return 0;
        }

        totalError += error;

        double output = getP(error) + getI(error) + getD(error, runtime);

        lastError = error;
        errors.add(output);
        lastTime = runtime.time();

        return output;
    }

    public ArrayList<Double> aman(){
        return errors;
    }
}
