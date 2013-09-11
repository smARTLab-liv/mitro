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
public class AEC {

    private AudioFilter.LP_5 lp;
    private AudioFilter.FIR_HP_13 hp0;
    private AudioFilter.IIR_HP hp00;
    private AudioFilter.IIR_HP hp1;
    private AudioFilter.Geigel_DTD dtd;
    private AEC_NLMS_PW nlms;
    private double s0avg;

    public AEC(){
        hp0=new AudioFilter.FIR_HP_13();
        hp00=new AudioFilter.IIR_HP();
        hp1=new AudioFilter.IIR_HP();
        dtd=new AudioFilter.Geigel_DTD();
        lp=new AudioFilter.LP_5();
        nlms=new AEC_NLMS_PW();
    }

    public int doAEC(int d, int x)
    {
          double s0 = (double)d;
          double s1 = (double)x;

          // Mic Highpass Filter - to remove DC
          s0 = hp00.IIR_HP(s0);

          // Mic Highpass Filter - telephone users are used to 300Hz cut-off
          s0 = hp0.FIR_HP_13(s0);

          //Apply Low pass to microphone
          //s0 = lp.LP_5(s0);

          // ambient mic level estimation
          s0avg += 1e-4f*(Math.abs(s0) - s0avg);

          // Spk Highpass Filter - to remove DC
          s1 = hp1.IIR_HP(s1);

          // Double Talk Detector
          boolean update = !dtd.GEIGEL_DTD(s0, s1);

          // Acoustic Echo Cancellation
          s0 = nlms.NLMS_PW(s0, s1, update);

          // Acoustic Echo Suppression
          if (update) {
            // Non Linear Processor (NLP): attenuate low volumes
            s0 *= Constants.NLPAttenuation;
          }


          // Saturation
          if (s0 > Constants.MAXPCM) {
            return (int)Constants.MAXPCM;
          } else if (s0 < -Constants.MAXPCM) {
            return (int)-Constants.MAXPCM;
          } else {
            return (int)Math.round(s0);
          }
    }
}
