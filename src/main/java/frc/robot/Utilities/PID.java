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

	public void Update(double P, double I, double D, double sampleRate){
		p = P;
		i = I;
		d = D;
		this.sampleRate = sampleRate;
	}
    
    //use this for a general PID output
	public double Out(double currentValue, double targetValue, double targetRate){

		double error = targetValue - currentValue;

		rate = (currentValue - lastValue)/sampleRate;

		double rateError = targetRate - rate;
        
        accumulatedError = accumulatedError + error;
		
        lastValue = currentValue;

		return p*error + i*accumulatedError + d*rateError;
	}

    //use this if you are getting a rate value as input like from the gyroscope
	public double RateOut(double currentValue, double targetValue, double targetRate, double rateInput){

		double error = targetValue - currentValue;
        
		rate = rateInput;

		double rateError = targetRate - rate;
        
        accumulatedError = accumulatedError + error;
		
        lastValue = currentValue;

		return p*error + i*accumulatedError + d*rateError;
	}

	public double getRate(){
		return rate;
	}
}
