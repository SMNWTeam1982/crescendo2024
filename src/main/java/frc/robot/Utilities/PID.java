package frc.robot.Utilities;

public class PID {
	////PID loops are used to get something to a certain position
	////they are too complicated to talk about here ask one of the mentors to explain or look it up on the internet
    private double lastValue = 0;
	private double sampleRate = 0.02;
	private double p;
	private double i;
	private double d;
	private double accumulatedError = 0;
	private double rate = 0;

	public PID(double P,double I, double D){
		p = P;
		i = I;
		d = D;
	}

	public void update(double P, double I, double D){
		p = P;
		i = I;
		d = D;
	}
    
    //use this for a general PID output
	public double out(double currentValue, double targetValue, double targetRate){

		double error = targetValue - currentValue;

		rate = (currentValue - lastValue)/sampleRate;

		double rateError = targetRate - rate;
        
        accumulatedError += error;
		
        lastValue = currentValue;

		return p*error + i*accumulatedError + d*rateError;
	}

    //use this if you are getting a rate value as input like from the gyroscope
	public double rate_out(double currentValue, double targetValue, double targetRate, double rateInput){

		double error = targetValue - currentValue;
        
		rate = rateInput;

		double rateError = targetRate - rate;
        
        accumulatedError += error;
		
        lastValue = currentValue;

		return p*error + i*accumulatedError + d*rateError;
	}

	public double get_rate(){
		return rate;
	}
}
