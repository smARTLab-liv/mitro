/*
Copyright (C) 2011

This file is part of JLZ77
written by Max Bügler
http://www.maxbuegler.eu/

JLZ77 is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option) any
later version.

JLZ77 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
package LZ77;

import Common.Basis;
import Common.BitOutputStream;
import Main.Constants;

import java.io.IOException;
import java.io.OutputStream;

/**
 * Class to compress a stream
 */
public class LZ77Compressor extends OutputStream {
    private BitOutputStream out;
    private LZWindow win;
    private LZSequence seq;
    private int windowsize;
    private boolean match=false;
    private int shift=0;
    private long countin, countout;

    public LZ77Compressor(OutputStream out) {
        this.out = new BitOutputStream(out);
        reset();
    }

    public void reset(){
        win=new LZWindow(Constants.AUDIO_LZ77_STREAM_BITS);
        windowsize =(int)Math.pow(2, Constants.AUDIO_LZ77_STREAM_BITS);
        countin=0;
        countout=0;
    }

    public void write(int i) throws IOException {
        countin+=8;
        byte b=(byte)i;

        if (win.size()==0){
            //Write shift and length 0
            out.write(Basis.intToBooleans(0, Constants.AUDIO_LZ77_STREAM_BITS));
            out.write(Basis.intToBooleans(0, Constants.AUDIO_LZ77_STREAM_BITS));

            //Write read byte
            out.write(b);

            countout+=2*Constants.AUDIO_LZ77_STREAM_BITS+8;
            //Add byte to window
            win.add(b);
            //System.out.println("000");
        }
        else{
            if (!match){
                //Start sequence with read byte
                seq=new LZSequence(windowsize +1);
                shift=0;
                seq.add(b);
                //System.out.println("!M");
            }

            //Check if sequence is contained in window
            int iiw=seq.isInWindow(win);

            if (iiw<0){
                //System.out.println("M!M");
                //Write shift and length of matching to file
                out.write(Basis.intToBooleans(shift, Constants.AUDIO_LZ77_STREAM_BITS));
                out.write(Basis.intToBooleans(seq.size()-1, Constants.AUDIO_LZ77_STREAM_BITS));

                //Write last byte of sequence
                out.write(seq.getLast());

                countout+=2*Constants.AUDIO_LZ77_STREAM_BITS+8;

                //Add sequence to window
                for (int x=0;x<seq.size();x++)win.add(seq.get(x));
                
                match=false;
            }
            else{
                //System.out.println("MMM");
                match=true;
                shift=iiw;
                seq.add(b);
            }
        }
        if (countin%100000==0)System.out.println(countout+"/"+countin+" = "+(double)countout/countin); 
    }
}
