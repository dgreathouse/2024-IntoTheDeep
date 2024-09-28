package org.firstinspires.ftc.teamcode.Lib;

public abstract class RobotDrive {
    protected double m_rangeMin = -1.0;
    protected double m_rangeMax = 1.0;
    protected double m_maxOutput = 1.0;

    public RobotDrive(){

    }
    public void setMaxOutput(double _maxOutput) {
        this.m_maxOutput = _maxOutput;
    }
    public void range(double _min, double _max) {
        this.m_rangeMin = _min;
        this.m_rangeMax = _max;
    }
    /**
     * Returns minimum range value if the given value is less than
     * the set minimum. If the value is greater than the set maximum,
     * then the method returns the maximum value.
     *
     * @param value The value to clip.
     */
    public double clipRange(double value) {
        return value <= m_rangeMin ? m_rangeMin
                : value >= m_rangeMax ? m_rangeMax
                : value;
    }
    public abstract void stop();

    /** <b>Normalize the speeds to 1 and scale them from 0.0 to 1.0</b>
     *
     * @param wheelSpeeds The array of wheel speeds
     * @param magnitude The magnitude to multiply the speeds by after normalizing.
     */
    protected void normalize(double[] wheelSpeeds, double magnitude) {
        // Find the maximum speed
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        // If greater than 1 or max speed, normalize to a max of 1.
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }
        // Scale all values based on the requested magnitude
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i]) * magnitude;
        }
    }
    protected void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }

    }
    /**
     * Square magnitude of number while keeping the sign.
     */
    protected double squareInput(double input) {
        return input * Math.abs(input);
    }

}
