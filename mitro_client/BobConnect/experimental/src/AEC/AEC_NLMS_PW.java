package AEC;

/**
 * Adaptive Echo cancellation
 * Adapted from
 * http://www.andreadrian.de/echo_cancel/draft-aec-03.txt
 */
public class AEC_NLMS_PW {

    /* maximum NLMS filter length in taps. A longer filter length gives
     * better Echo Cancellation, but slower convergence speed and
     * needs more CPU power (Order of NLMS is linear) */
    private final int NLMS_LEN=80*8;


    // Extention in taps to reduce mem copies
    private final int NLMS_EXT=10*8;

    /* convergence speed. Range: >0 to <1 (0.2 to 0.7). Larger values give
     * more AEC in lower frequencies, but less AEC in higher frequencies. */
    private final double  Stepsize = 0.7f;

    private AudioFilter.SP_IIR_HP f1=new AudioFilter.SP_IIR_HP();
    private AudioFilter.SP_IIR_HP f2=new AudioFilter.SP_IIR_HP();
    
    private double[] nlms_x=new double[NLMS_LEN + NLMS_EXT];  // tap delayed loudspeaker signal
    private double[] nlms_xf=new double[NLMS_LEN + NLMS_EXT]; // pre-whitening tap delayed signal
    private double[] nlms_w=new double[NLMS_LEN];             // tap weights
    private int nlms_lastupdate;               // optimize: iterative dotp(x,x)
    private double nlms_dotp_xf_xf;            // double to avoid loss of precision
    private double nlms_Min_dotp_xf_xf;
    private double nlms_s0avg;
    private int nlms_j;

    public double NLMS_PW(double mic, double spk, boolean update){
        double d = mic;      	        // desired signal
        nlms_x[nlms_j] = spk;
        nlms_xf[nlms_j] = f1.SP_IIR_HP(spk);     // pre-whitening of x

        // calculate error value
        // (mic signal - estimated mic signal from spk signal)

        double e=0;
        for (int x=0;x<nlms_w.length;x++){
          e+=nlms_w[x]*nlms_x[x+nlms_j];
        }
        e=d-e;

        double ef = f2.SP_IIR_HP(mic);   // pre-whitening of e
        // optimize: iterative dotp(xf, xf)
        nlms_dotp_xf_xf += (nlms_xf[nlms_j]*nlms_xf[nlms_j] - nlms_xf[nlms_j+NLMS_LEN-1]*nlms_xf[nlms_j+NLMS_LEN-1]);

        if (update) {
            // calculate variable step size
            double mikro_ef = Stepsize * ef / nlms_dotp_xf_xf;

            // update tap weights (filter learning)
            for (int i = 0; i < NLMS_LEN; i += 2) {
              // optimize: partial loop unrolling
              nlms_w[i] += mikro_ef*nlms_xf[i+nlms_j];
              nlms_w[i+1] += mikro_ef*nlms_xf[i+nlms_j+1];
            }
        }

        if (--nlms_j < 0) {
        // optimize: decrease number of memory copies
        nlms_j = NLMS_EXT;
        for (int x=0;x<NLMS_LEN-1;x++){
            nlms_x[x]=nlms_x[x+nlms_j+1];
            nlms_xf[x]=nlms_xf[x+nlms_j+1];
        }
        }

        return e;
    }
}
