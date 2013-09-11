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
package LZ77;

import Common.Basis;
import Common.BitInputStream;
import Common.BitOutputStream;
import Main.Constants;

import java.io.*;

/**
 * A Java implementation of the LZ77 compression algorithm as published by Abraham Lempel and Jacob Ziv
 */
public class LZ77 {
    private InputStream in;
    private OutputStream out;
    private BitInputStream bIn;
    private BitOutputStream bOut;
    private boolean stop,compress;

    public LZ77(OutputStream out, boolean compress){
        this.in=in;
        this.out=out;
        this.compress=compress;
        if (compress){
            bOut=new BitOutputStream(out);
        }
        else{
            bIn=new BitInputStream(in);
            bOut=new BitOutputStream(out);
        }
    }


    public void start(){
        stop=false;
        new LZ77Thread().start();
    }

    public void stop(){
        stop=true;
    }

    class LZ77Thread extends Thread{

        public void run(){
            while(!stop){
                int read=0;
                try{
                    //Calculate buffersize in bytes
                    int buffersize=(int)Math.pow(2, Constants.AUDIO_LZ77_STREAM_BITS);

                    //Initialize Window
                    LZWindow win=new LZWindow(buffersize);

                    if (compress){
                        //Read first byte
                        byte[] readbuffer=new byte[1];

                        read=1;
                        //While there is stuff to read
                        while(!stop){

                            //Read byte
                            read=in.read(readbuffer);

                            //If we don't have data in our window
                            if (win.size()==0){

                                //Write shift and length 0
                                bOut.write(Basis.intToBooleans(0, Constants.AUDIO_LZ77_STREAM_BITS));
                                bOut.write(Basis.intToBooleans(0, Constants.AUDIO_LZ77_STREAM_BITS));

                                //Write read byte
                                bOut.write(readbuffer);

                                //Add byte to window
                                win.add(readbuffer[0]);
                            }
                            else{ //Search for matching

                                //Start sequence with read byte
                                LZSequence seq=new LZSequence(buffersize +1);
                                seq.add(readbuffer[0]);

                                boolean match=true;
                                int shift=0;

                                //While there is a match
                                while (match){

                                    //Check if sequence is contained in window
                                    int iiw=seq.isInWindow(win);

                                    //If not contained or no more data available
                                    if (iiw<0||in.available()<1)match=false;

                                    //If match found and new data available
                                    else{

                                        //Set shift value
                                        shift=iiw;

                                        //Read next byte
                                        read=in.read(readbuffer);

                                        //If nothin read we stop
                                        //if (read==0)break;

                                        //Add new byte to sequence
                                        seq.add(readbuffer[0]);
                                    }
                                }

                                //Write shift and length of matching to file
                                bOut.write(Basis.intToBooleans(shift, Constants.AUDIO_LZ77_STREAM_BITS));
                                bOut.write(Basis.intToBooleans(seq.size()-1, Constants.AUDIO_LZ77_STREAM_BITS));

                                //Write last byte of sequence
                                bOut.write(seq.getLast());

                                //Add sequence to window
                                for (int x=0;x<seq.size();x++)win.add(seq.get(x));
                            }
                            if (read==0)Thread.sleep(1);
                        }

                    }
                    else{
                        //While there is data to read
                        while(!stop){

                            //Initialize buffer for shift and length of matching
                            boolean[] bitbuffer=new boolean[Constants.AUDIO_LZ77_STREAM_BITS];

                            //Read shift
                            read=bIn.read(bitbuffer);

                            //If something was read
                            if (read>0){

                                //Calculate shift
                                int shift=Basis.booleansToInt(bitbuffer);

                                //Read length of matching
                                bIn.read(bitbuffer);
                                int length=Basis.booleansToInt(bitbuffer);

                                //Read byte
                                byte b=bIn.readByte();

                                //If there was a matching
                                if (length>0){

                                    //Create buffer for matched sequence
                                    byte[] temp=new byte[length];

                                    //Read sequence from window
                                    int y=0;
                                    for (int x=shift;x>shift-length;x--){
                                        byte wb=win.get(x);
                                        bOut.write(wb);
                                        temp[y++]=wb;
                                    }

                                    //Add sequence to window
                                    for (int x=0;x<length;x++)win.add(temp[x]);
                                }

                                //Write read (non matching) byte
                                bOut.write(b);

                                //Add byte to window
                                win.add(b);

                            }
                            if (read==0)Thread.sleep(1);
                        }


                    }


                }
                catch(Exception ex){
                    stop=true;
                }
            }
        }
    }

    /**
     * Compresesses File in to File out using a window of specified bits
     * @param in input file
     * @param out output file
     * @param bits window size (2^bits bytes)
     * @throws FileNotFoundException
     * @throws IOException
     */
    public static void compress(File in, File out, int bits) throws FileNotFoundException, IOException {

        //Calculate buffersize in bytes
        int buffersize=(int)Math.pow(2,bits);

        //Initialize Window
        LZWindow win=new LZWindow(buffersize);

        //Initialize streams
        FileInputStream fis=new FileInputStream(in);
        BitOutputStream bos=new BitOutputStream(out);

        //Write buffersize
        bos.write(Basis.intToBooleans(bits,6));

        //Read first byte
        byte[] readbuffer=new byte[1];

        int read=1;

        //While there is stuff to read
        while(read>0&&fis.available()>0){

            //Read byte
            read=fis.read(readbuffer);

            //If we don't have data in our window
            if (win.size()==0){

                //Write shift and length 0
                bos.write(Basis.intToBooleans(0, bits));
                bos.write(Basis.intToBooleans(0, bits));

                //Write read byte
                bos.write(readbuffer);

                //Add byte to window
                win.add(readbuffer[0]);
            }
            else{ //Search for matching

                //Start sequence with read byte
                LZSequence seq=new LZSequence(buffersize +1);
                seq.add(readbuffer[0]);

                boolean match=true;
                int shift=0;

                //While there is a match
                while (match){

                    //Check if sequence is contained in window
                    int iiw=seq.isInWindow(win);

                    //If not contained or no more data available
                    if (iiw<0||fis.available()<1)match=false;

                    //If match found and new data available
                    else{

                        //Set shift value
                        shift=iiw;

                        //Read next byte
                        read=fis.read(readbuffer);

                        //If nothin read we stop
                        if (read==0)break;

                        //Add new byte to sequence
                        seq.add(readbuffer[0]);
                    }
                }

                //Write shift and length of matching to file
                bos.write(Basis.intToBooleans(shift, bits));
                bos.write(Basis.intToBooleans(seq.size()-1, bits));

                //Write last byte of sequence
                bos.write(seq.getLast());

                //Add sequence to window
                for (int x=0;x<seq.size();x++)win.add(seq.get(x));
            }
        }

        //Close streams
        bos.close();
        fis.close();
    }


    /**
     * Decompresses file in and saves it to out
     * @param in input file
     * @param out output file
     * @throws FileNotFoundException
     * @throws IOException
     */
    public static void decompress(File in, File out) throws FileNotFoundException, IOException {

        //Initialize streams
        BitInputStream bis=new BitInputStream(in);
        BitOutputStream bos=new BitOutputStream(out);

        //Setup buffer for metadata
        boolean[] bitbuffer=new boolean[6];

        //Read number of bits for window
        bis.read(bitbuffer);
        int bits=Basis.booleansToInt(bitbuffer);

        //Calculate buffersize
        int buffersize=(int)Math.pow(2,bits);

        //Initialize window
        LZWindow win=new LZWindow(buffersize);

        int read=1;

        //While there is data to read
        while(read>0&&bis.available()>0){

            //Initialize buffer for shift and length of matching
            bitbuffer=new boolean[bits];

            //Read shift
            read=bis.read(bitbuffer);

            //If something was read
            if (read>0){

                //Calculate shift
                int shift=Basis.booleansToInt(bitbuffer);

                //Read length of matching
                bis.read(bitbuffer);
                int length=Basis.booleansToInt(bitbuffer);

                //Read byte
                byte b=bis.readByte();

                //If there was a matching
                if (length>0){

                    //Create buffer for matched sequence
                    byte[] temp=new byte[length];

                    //Read sequence from window
                    int y=0;
                    for (int x=shift;x>shift-length;x--){
                        byte wb=win.get(x);
                        bos.write(wb);
                        temp[y++]=wb;
                    }

                    //Add sequence to window
                    for (int x=0;x<length;x++)win.add(temp[x]);
                }

                //Write read (non matching) byte
                bos.write(b);

                //Add byte to window
                win.add(b);

            }
        }

        //Close streams
        bos.close();
        bis.close();
    }

    public static void main(String[] args) throws Exception{
        compress(new File("/home/homer/huffExp/test"),new File("/home/homer/huffExp/lztest4"),12);
        decompress(new File("/home/homer/huffExp/lztest4"),new File("/home/homer/huffExp/lztest5"));
    }
}
