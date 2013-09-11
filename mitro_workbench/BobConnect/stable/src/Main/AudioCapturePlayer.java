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

package Main;

import AEC.*;

import javax.sound.sampled.*;
import java.io.InputStream;
import java.io.OutputStream;

/**
 * Plays and captures audio while filtering the playback audio from the mic input
 */
public class AudioCapturePlayer {
    private OutputStream out;
    private InputStream in;
    private AudioFormat format;
    private TargetDataLine tLine;
    private SourceDataLine sLine;
    private AEC filter;
    private boolean stop;

    public AudioCapturePlayer(InputStream in, OutputStream out){
        this.in=in;
        this.out=out;
        this.format=new AudioFormat(Constants.AUDIO_SAMPLE_RATE,Constants.AUDIO_SAMPLE_BITS, Constants.AUDIO_CHANNELS, Constants.AUDIO_SIGNED, Constants.AUDIO_BIG_ENDIAN);
    }

    public void start() throws Exception{
        filter=new AEC();
        DataLine.Info info = new DataLine.Info(SourceDataLine.class, format);
        sLine = (SourceDataLine) AudioSystem.getLine(info);
        sLine.open(format);
        sLine.start();

        info = new DataLine.Info(TargetDataLine.class, format);
        tLine=(TargetDataLine)AudioSystem.getLine(info);
        tLine.open(format);
        tLine.start();
        stop=false;
        new AudioPlayerCaptureThread().start();
    }

    public void stop(){
        this.stop=true;
    }

    class AudioPlayerCaptureThread extends Thread{
        public void run(){
            int buffersize=1000;
            byte[] bufferPlay=new byte[buffersize];//Constants.AUDIO_BUFFER_SIZE];
            byte[] bufferRec=new byte[buffersize];//Constants.AUDIO_BUFFER_SIZE];
            boolean start=true;
            AudioFilter.LP_5 lpf=new AudioFilter.LP_5();
            int counter=0;
            int prebuffer=0;
            while(!stop){
                try{
                    if (!start){

                        //if (prebuffer<10000){
                        //    prebuffer++;
                        //}
                        //else{
                            //Read from stream
                        in.read(bufferPlay,0,buffersize);
                        //}

                        //Write to playback
                        sLine.write(bufferPlay,0,buffersize);

                        //Read from mic
                        tLine.read(bufferRec,0,buffersize);



                    }
                    else{

                        //Read from stream
                        //in.read(bufferPlay,count,1);

                        //Read from mic
                        tLine.read(bufferRec,0,buffersize);
                    }

                    for (int x=0;x<buffersize;x++){
                        //AEC the shit out of it here
                        bufferRec[x]=(byte)filter.doAEC(bufferRec[x],bufferPlay[x]);
                        bufferRec[x]=(byte)lpf.LP_5(bufferRec[x]);
                    }

                    //Write to stream
                    out.write(bufferRec,0,buffersize);

                    if (start) out.flush();
                    
                    counter++;

                    //start=false;

                    if (buffersize*counter>Constants.AUDIO_BUFFER_SIZE-110){

                        //We have received the first packet
                        start=false;

                        counter=0;

                        out.flush();
                    }

                    /*//If this is not the first packet
                    if (!start){

                        //Read from stream
                        count=in.read(bufferPlay);
                    }
                    else{

                        //Start with a packet of 0's
                        count=bufferRec.length;
                    }

                    //Read same number of bytes as received from microphone line
                    while(count2<count&&!stop){
                        count2+=tLine.read(bufferRec,count2,1);
                    }

                    //We have received the first packet
                    start=false;

                    //If we got any samples
                    if (count>0){
			
                        //AEC the shit out of it here
                        for (int x=0;x<count;x++){
                            bufferRec[x]=(byte)filter.doAEC(bufferRec[x],bufferPlay[x]);
                            //bufferPlay[x]=(byte)lpf.LP_5(bufferPlay[x]);
                        }

                        //Write to playback
                        sLine.write(bufferPlay,0,count);

                        //Write to outputstream
                        out.write(bufferRec,0,count);
                        out.flush();
                    }*/
                }
                catch(Exception ex){
                    //Probably a broken pipe
                    ex.printStackTrace();
                    stop=true;
                }
            }
            tLine.drain();
            sLine.drain();
            tLine.close();
            sLine.close();

        }
    }
}
