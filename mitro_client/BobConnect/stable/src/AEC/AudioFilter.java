/*
Copyright (C) 2011

This file is part of BobConnect
written by Max Bügler
http://www.maxbuegler.eu/

BobConnect is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option) any
later version.

BobConnect is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
package AEC;

/**
 * Adaptive Echo cancellation
 * Adapted from
 * http://www.andreadrian.de/echo_cancel/draft-aec-03.txt
 */
public class AudioFilter {

    public static class IIR_HP{
        private double iir_x=0;
        /* Exponential Smoothing or IIR Infinite Impulse Response Filter */
        public double IIR_HP(double in){
            /* Highpass = Signal - Lowpass. Lowpass = Exponential Smoothing */
            iir_x += Constants.IIR_A0 * (in - iir_x);
            return in - iir_x;
        }
    }

    public static class LP_5{
        private double[] lp_5_z=new double[]{0,0,0,0,0};
        private double[] lp_5_coeff=new double[]{0.2,0.2,0.2,0.2,0.2};
        private int lp_pointer=0;

        /* A low pass filter with 2 zeros to get rid of some noise
         *             
         */
        public double LP_5(double in){
            lp_5_z[lp_pointer]=in;
            lp_pointer++;
            if (lp_pointer>=lp_5_z.length)lp_pointer=0;
            double sum = 0.0;
            for (int x = 0; x < 5; x++) {
              sum += lp_5_coeff[x] * lp_5_z[(lp_pointer +x)%5];
            }

            return sum;
        }
    }

    public static class FIR_HP_13{
        private double[] fir_hp13_z=new double[]{0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        private int fir_pointer=0;

        /* 13 taps FIR Finite Impulse Response filter
        * Coefficients calculated with
        * www.dsptutor.freeuk.com/KaiserFilterDesign/KaiserFilterDesign.html
        */
        public double FIR_HP_13(double in){
            fir_hp13_z[fir_pointer]=in;
            fir_pointer++;
            if (fir_pointer>=fir_hp13_z.length)fir_pointer=0;
            double sum0 = 0.0, sum1 = 0.0;


            for (int x = 0; x < 14; x+= 2) {
              // optimize: partial loop unrolling
              sum0 += Constants.FIR_HP13_A[x] * fir_hp13_z[(fir_pointer+x)%13];
              sum1 += Constants.FIR_HP13_A[x+1] * fir_hp13_z[(fir_pointer+x+1)%13];
            }

            return sum0+sum1;
        }
    }

    public static class SP_IIR_HP{
        private double sp_iir_x=0,sp_iir_y=0;
        /* Recursive single pole IIR Infinite Impulse response filter
        * Coefficients calculated with
        * http://www.dsptutor.freeuk.com/IIRFilterDesign/IIRFiltDes102.html
        */
        public double SP_IIR_HP(double in){
            double out = Constants.SP_IIR_A0 * in + Constants.SP_IIR_A1 * sp_iir_x + Constants.SP_IIR_B1 * sp_iir_y;
            sp_iir_x = in;
            sp_iir_y = out;
            return out;
        }
    }

    public static class TP_IIR_HP{
        private double[] tp_iir_x=new double[]{0,0};
        private double[] tp_iir_y=new double[]{0,0};

        /* Recursive two pole IIR Infinite Impulse Response filter
        * Coefficients calculated with
        * http://www.dsptutor.freeuk.com/IIRFilterDesign/IIRFiltDes102.html
        */
        public double TP_IIR_HP(double in){
            double out =
                Constants.TP_IIR_A[0] * in +
                Constants.TP_IIR_A[1] * tp_iir_x[0] +
                Constants.TP_IIR_A[2] * tp_iir_x[1] -
                Constants.TP_IIR_B[0] * tp_iir_y[0] -
                Constants.TP_IIR_B[1] * tp_iir_y[1];

            tp_iir_x[1] = tp_iir_x[0];
            tp_iir_x[0] = in;
            tp_iir_y[1] = tp_iir_y[0];
            tp_iir_y[0] = out;
            return out;
        }
    }


    public static class Geigel_DTD{
        private int dtd_Cnt, dtd_Ndx, dtd_Hangover;
        private double dtd_x=0,dtd_max_max_x=0;
        private double[] dtd_max_x=new double[Constants.NLMS_LEN / Constants.DTD_LEN];

        /* Geigel Double-Talk Detector
         *
         * in mic: microphone sample (PCM as REALing point value)
         * in spk: loudspeaker sample (PCM as REALing point value)
         * return: 0 for no talking, 1 for talking
         */
        public boolean GEIGEL_DTD(double mic, double spk){
              // optimized implementation of max(|x[0]|, |x[1]|, .., |x[L-1]|):
              // calculate max of block (DTD_LEN values)
              dtd_x = Math.abs(dtd_x);
              if (dtd_x > dtd_max_x[dtd_Ndx]) {
                dtd_max_x[dtd_Ndx] = dtd_x;
                if (dtd_x > dtd_max_max_x) {
                  dtd_max_max_x = dtd_x;
                }
              }
              if (++dtd_Cnt >= Constants.DTD_LEN) {
                dtd_Cnt = 0;
                // calculate max of max
                dtd_max_max_x = 0.0f;
                for (int i = 0; i < Constants.NLMS_LEN/Constants.DTD_LEN; ++i) {
                  if (dtd_max_x[i] > dtd_max_max_x) {
                    dtd_max_max_x = dtd_max_x[i];
                  }
                }
                // rotate Ndx
                if (++dtd_Ndx >= Constants.NLMS_LEN/Constants.DTD_LEN) dtd_Ndx = 0;
                dtd_max_x[dtd_Ndx] = 0.0f;
              }

              // The Geigel DTD algorithm with Hangover timer Thold
              if (Math.abs(mic) >= Constants.GeigelThreshold * dtd_max_max_x) {
                dtd_Hangover = Constants.Thold;
              }

              if (dtd_Hangover > 0) --dtd_Hangover;

              return (dtd_Hangover > 0);
        }
    }






}
