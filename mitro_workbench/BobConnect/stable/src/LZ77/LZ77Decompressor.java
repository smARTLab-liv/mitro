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
import Main.Constants;

import java.io.IOException;
import java.io.InputStream;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Class to decomrpress a stream
 */
public class LZ77Decompressor extends InputStream {
    private BitInputStream in;
    private LZWindow win;
    private byte[] outBuffer;
    private int windowsize;
    private boolean[] bitbuffer;
    private boolean stop;
    private int pointer,start;
    private ReentrantLock lock;

    public LZ77Decompressor(InputStream in){
        this.in=new BitInputStream(in);
        this.outBuffer=new byte[Constants.AUDIO_LZ77_DECOMPRESOR_BUFFER_SIZE];
        this.win=new LZWindow(Constants.AUDIO_LZ77_STREAM_BITS);
        this.windowsize=(int)Math.pow(2, Constants.AUDIO_LZ77_STREAM_BITS);
        this.bitbuffer=new boolean[Constants.AUDIO_LZ77_STREAM_BITS];
        this.lock=new ReentrantLock();
        this.pointer=0;
        this.start=0;
        new DecompressorThread().start();
    }

    public void stop(){
        this.stop=true;
    }

    class DecompressorThread extends Thread{
        public void run(){
            int read;
            while(!stop){
                try{
                    //Read shift
                    read=in.read(bitbuffer);

                    //System.out.println(read);

                    //If something was read
                    if (read>0){

                        //Calculate shift
                        int shift= Basis.booleansToInt(bitbuffer);

                        //Read length of matching
                        in.read(bitbuffer);
                        int length=Basis.booleansToInt(bitbuffer);

                        //Read byte
                        byte b=in.readByte();

                        //If there was a matching
                        if (length>0){

                            //Create buffer for matched sequence
                            byte[] temp=new byte[length];

                            //Read sequence from window
                            int y=0;
                            for (int x=shift;x>shift-length;x--){
                                byte wb=win.get(x);
                                write(wb);
                                temp[y++]=wb;
                            }

                            //Add sequence to window
                            for (int x=0;x<length;x++)win.add(temp[x]);
                        }

                        //Write read (non matching) byte
                        write(b);

                        //Add byte to window
                        win.add(b);

                    }
                }
                catch(Exception ex){
                    ex.printStackTrace();
                    stop=true;
                }
            }
        }
    }


    public void write(byte b){
        try{
            int next=pointer+1;
            if (next>=outBuffer.length)next=0;
            if (next==start)
                System.out.println("BUFFER OVERFLOW");
            else{
                outBuffer[pointer]=b;
                pointer++;
                if (pointer>=outBuffer.length)pointer=0;
            }

        }
        catch(Exception ex){
            ex.printStackTrace();
        }
    }

    public int read() throws IOException {
        byte res;
        if (pointer!=start){
            start++;
            if (start>=outBuffer.length)start=0;
            res=outBuffer[start];
        }
        else{
            while(pointer==start){
                try{
                    Thread.sleep(1);
                }
                catch(Exception ex){}
            }
            start++;
            if (start>=outBuffer.length)start=0;
            res=outBuffer[start];
        }
        return res;
    }
}
