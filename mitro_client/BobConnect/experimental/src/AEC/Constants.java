package AEC;

/**
 * Adaptive Echo cancellation
 * Adapted from
 * http://www.andreadrian.de/echo_cancel/draft-aec-03.txt
 */
public class Constants {
    public static final double IIR_A0=0.01;
    public static final double[] FIR_HP13_A=new double[]{
      // Kaiser Window FIR Filter, Filter type: High pass
      // Passband: 300.0 - 4000.0 Hz, Order: 12
      // Transition band: 100.0 Hz, Stopband attenuation: 10.0 dB
      -0.043183226f, -0.046636667f, -0.049576525f, -0.051936015f,
      -0.053661242f, -0.054712527f, 0.82598513f, -0.054712527f,
      -0.053661242f, -0.051936015f, -0.049576525f, -0.046636667f,
      -0.043183226f, 0.0f
    };
    
    // Chebyshev IIR filter, Filter type: HP
    // Passband: 3700 - 4000.0 Hz
    // Passband ripple: 1.5 dB, Order: 1
    public static final double SP_IIR_A0 = 0.105831884f;
    public static final double SP_IIR_A1 = -0.105831884;
    public static final double SP_IIR_B1 = 0.78833646f;
    
    // Butterworth IIR filter, Filter type: HP
    // Passband: 2000 - 4000.0 Hz, Order: 2
    public static final double[] TP_IIR_A = { 0.29289323f, -0.58578646f, 0.29289323f };
    public static final double[] TP_IIR_B = { 1.3007072E-16f, 0.17157288f };
    
    public static final double[] LP_COEFF = { 1.0, -0.049, 1.1581, -0.049, 1.029 };
    
    
    /* dB Values */
    public static final double  M0dB = 1.0f;
    public static final double  M3dB = 0.71f;
    public static final double  M6dB = 0.50f;
    public static final double  M9dB = 0.35f;
    public static final double  M12dB = 0.25f;
    public static final double  M18dB = 0.125f;
    public static final double  M24dB = 0.063f;
    
    /* dB values for 16bit PCM */
    /* MxdB_PCM = 32767 * 10 ^(x / 20) */
    public static final double  M10dB_PCM = 10362.0f;
    public static final double  M20dB_PCM = 3277.0f;
    public static final double  M25dB_PCM = 1843.0f;
    public static final double  M30dB_PCM = 1026.0f;
    public static final double  M35dB_PCM = 583.0f;
    public static final double  M40dB_PCM = 328.0f;
    public static final double  M45dB_PCM = 184.0f;
    public static final double  M50dB_PCM = 104.0f;
    public static final double  M55dB_PCM = 58.0f;
    public static final double  M60dB_PCM = 33.0f;
    public static final double  M65dB_PCM = 18.0f;
    public static final double  M70dB_PCM = 10.0f;
    public static final double  M75dB_PCM = 6.0f;
    public static final double  M80dB_PCM = 3.0f;
    public static final double  M85dB_PCM = 2.0f;
    public static final double  M90dB_PCM = 1.0f;
    
    public static final double MAXPCM = 127;
    
    /* Design constants (Change to fine tune the algorithms */
    
    /* The following values are for hardware AEC and studio quality
     * microphone */
    
    /* maximum NLMS filter length in taps. A longer filter length gives
     * better Echo Cancellation, but slower convergence speed and
     * needs more CPU power (Order of NLMS is linear) */
    public static final int NLMS_LEN=80*8;
    
    /* convergence speed. Range: >0 to <1 (0.2 to 0.7). Larger values give
     * more AEC in lower frequencies, but less AEC in higher frequencies. */
    public static final double  Stepsize = 0.7f;
    
    /* minimum energy in xf. Range: M70dB_PCM to M50dB_PCM. Should be equal
     * to microphone ambient Noise level */
    public static final double  Min_xf = M75dB_PCM;
    
    /* Double Talk Detector Speaker/Microphone Threshold. Range <=1
     * Large value (M0dB) is good for Single-Talk Echo cancellation,
     * small value (M12dB) is good for Doulbe-Talk AEC */
    public static final double  GeigelThreshold = M6dB;
    
    /* Double Talk Detector hangover in taps. Not relevant for Single-Talk
     * AEC */
    public static final int Thold = 30 * 8;
    
    /* for Non Linear Processor. Range >0 to 1. Large value (M0dB) is good
     * for Double-Talk, small value (M12dB) is good for Single-Talk */
    public static final double  NLPAttenuation = M12dB;
    
    // Extention in taps to reduce mem copies
    public static final int NLMS_EXT=10*8;
    
    // block size in taps to optimize DTD calculation
    public static final int DTD_LEN=16;


    
}
