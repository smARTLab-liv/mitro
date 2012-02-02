/*
Copyright (C) 2011

This file is part of JLZ77
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

import javax.imageio.ImageIO;
import javax.imageio.ImageReader;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.BufferedInputStream;
import java.io.ByteArrayInputStream;

public class ImageStreamDecoder {
    private BufferedInputStream in;
    private ImageReader reader;
    private ImageStreamDecoderListener list;
	private final byte BASIS_B1=125;
	private final byte BASIS_B2=126;
	private final byte BASIS_TERM=127;
    private ReadThread thread;
    private boolean keyframe;
    private BufferedImage bufferImage, lastimg;
    //private BufferedImage img;

    public ImageStreamDecoder(BufferedInputStream in,ImageStreamDecoderListener list) {
        this.in = in;
        this.list = list;
        this.keyframe=false;
        ImageIO.setUseCache(false);
        int captureHeight=9*Constants.CAPUTRE_WIDTH/16;
        bufferImage=new BufferedImage(Constants.CAPUTRE_WIDTH,captureHeight,BufferedImage.TYPE_3BYTE_BGR);
    }

    public void start(){
        thread=new ReadThread();
        thread.start();
    }

    public void stop(){
        thread.stopIt();
    }

    class ReadThread extends Thread{
        private boolean stop;

        public void stopIt(){
            stop=true;
        }
        public void run(){
            long start=System.currentTimeMillis();
	        lastimg=null;
            try{

                stop=false;
                byte[] buffer=new byte[1000000];
                int read=in.read(buffer);
                while (!stop&&read>0){
                    for (int x=0;x<read;x++){
                        if (buffer[x]==BASIS_TERM){
                            if (x>0){
                                BufferedImage img=ImageIO.read(new ByteArrayInputStream(restoreByte(buffer,BASIS_B1,BASIS_B2,BASIS_TERM,x)));
                                if (keyframe==false && lastimg!=null){
                                    ImageTools.add(img,lastimg,bufferImage);
                                    img=bufferImage;
                                    //System.out.print(":DD:");
                                }
                                else{
                                    //System.out.print(":DK:");
                                }


                                if (System.currentTimeMillis()-start>1000){
                                    start=System.currentTimeMillis();
                                }
                                if (img!=null){
					                list.nextImage(img);
					                if (keyframe)
                                        lastimg=img;
				                }
                                else{
                                    System.out.println("DECODE FAIL");
                                }
                            }
                            for (int y=x+1;y<read;y++){
                                buffer[y-(x+1)]=buffer[y];
                            }
                            read-=(x+1);
                            x=-1;
                        }
                    }
                    try{
                        read+=in.read(buffer,read,buffer.length-read);
                    }
                    catch(Exception ex){
                        ex.printStackTrace();
                        stop=true;
                    }

                }

            }
            catch (Exception ex){
                ex.printStackTrace();
            }
        }
    }
/**
     * Restore freed byte
     * @param data input
     * @param b1 byte 1
     * @param b2 byte 2
     * @param free free byte
     * @return restored byte[]
     */
    public byte[] restoreByte(byte[] data,byte b1, byte b2, byte free, int length){
        keyframe=data[0]==1;
        byte[] temp=new byte[length];
        int pointer=0;
        for (int x=1;x<length;x++){
            if (data[x]==b2){
                x++;
                if (x<length&&data[x]==b1){
                    temp[pointer++]=free;
                }
            }
            else if (data[x]==b1){
                x++;
                if (x<length&&data[x]==b1){
                    temp[pointer++]=b1;
                }
                else if (data[x]==b2){
                    temp[pointer++]=b2;
                }

            }
            else{
                temp[pointer++]=data[x];
            }
        }
        byte[] out=new byte[pointer];
        for (int x=0;x<pointer;x++){
            out[x]=temp[x];
        }
        return out;
    }
}
