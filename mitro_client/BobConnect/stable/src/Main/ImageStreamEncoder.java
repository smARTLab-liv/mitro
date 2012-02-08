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

import javax.imageio.*;
import javax.imageio.plugins.jpeg.JPEGImageWriteParam;
import javax.imageio.stream.ImageOutputStream;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.Iterator;
import java.util.Locale;

public class ImageStreamEncoder {
    private ImageWriteParam iwparam=new MyImageWriteParam();
	private ImageWriter writer=null, writer2=null;
	private final byte BASIS_B1=125;
	private final byte BASIS_B2=126;
	private final byte BASIS_TERM=127;
    private OutputStream out;
    private boolean lock=false;
    private boolean writefail=false;
    private BufferedImage bufferImage, lastimg;
    private int keyframecounter;
    private ImageReader reader;

    public ImageStreamEncoder(OutputStream out) {
        this.out = out;
        keyframecounter=0;
        Iterator<ImageWriter> iter = ImageIO.getImageWritersByFormatName("jpg");
        if (iter.hasNext()) {
            writer = iter.next();
        }
        iter = ImageIO.getImageWritersByFormatName("png");
        if (iter.hasNext()) {
            writer2 = iter.next();
        }

	    ImageIO.setUseCache(false);


        int captureHeight=9*Constants.CAPUTRE_WIDTH/16;
        bufferImage=new BufferedImage(Constants.CAPUTRE_WIDTH,captureHeight,BufferedImage.TYPE_3BYTE_BGR);

    }

    public boolean hasWriteFailed(){
        return writefail;
    }

    public void encode(BufferedImage in,float quality, int mode){
        if (lock)return;
        lock=true;
        try{
            keyframecounter++;
            double diff=ImageTools.getDifference(in,lastimg);

            boolean keyframe=false;
            //System.out.println(diff);
            if (diff>100 || keyframecounter>25 || lastimg==null){
                System.out.print(":EK:");
                out.write(new byte[]{1});
                keyframecounter=0;
                keyframe=true;
                //lastimg=in;
                iwparam.setCompressionQuality(1f);
                //iwparam.setCompressionMode(ImageWriteParam.MODE_DEFAULT);
            }
            else{
                //System.out.print(":ED:");
                ImageTools.substract(in,lastimg,bufferImage);
                //lastimg=in;
                in=bufferImage;

                keyframe=false;

                //ImageTools.add(bufferImage,lastimg,lastimg);
                out.write(new byte[]{0});
                iwparam.setCompressionQuality(quality);

                //iwparam.setCompressionMode(ImageWriteParam.MODE_DEFAULT);
            }
            iwparam.setCompressionMode(mode);

            final ByteArrayOutputStream bos=new ByteArrayOutputStream(1000000);

            try{
                ImageOutputStream ios = ImageIO.createImageOutputStream(new OutputStream() {
                    public void write(int b) throws IOException {
                        bos.write(b);
                        //Free terminator byte
                        try{
                            if (b == BASIS_B1) {
                                out.write(new byte[]{BASIS_B1, BASIS_B1});
                            } else if (b == BASIS_B2) {
                                out.write(new byte[]{BASIS_B1, BASIS_B2});
                            } else if (b == BASIS_TERM) {
                                out.write(new byte[]{BASIS_B2, BASIS_B1});
                            } else {
                                out.write(b);
                            }
                            writefail=false;
                        }
                        catch(Exception ex){
                            writefail=true;
                        }
                    }
                });
                /*if (keyframe){
                    writer2.setOutput(ios);
                    writer2.write(null, new IIOImage(in, null, null), iwparam);
                }
                else{*/
                    writer.setOutput(ios);
                    writer.write(null, new IIOImage(in, null, null), iwparam);
                //}
                try{
                    out.write(new byte[]{BASIS_TERM,BASIS_TERM,BASIS_TERM});
                    out.flush();
                }
                catch(Exception ex){
                    writefail=true;
                }

                lock=false;
                if (keyframe)
                    lastimg=ImageIO.read(new ByteArrayInputStream(bos.toByteArray()));
                else{
                    //ImageTools.add(ImageIO.read(new ByteArrayInputStream(bos.toByteArray())),lastimg,lastimg);
                }
            }
            catch(Exception ex){
                ex.printStackTrace();
            }
        }
        catch(Exception ex){
            ex.printStackTrace();
        }
        lock=false;
	}


	public static class MyImageWriteParam extends JPEGImageWriteParam {
        public MyImageWriteParam() {
            super(Locale.getDefault());
        }

        public void setCompressionQuality(float quality) {
            if (quality < 0.0F || quality > 1.0F) {
                throw new IllegalArgumentException("Quality out-of-bounds!");
            }
            this.compressionQuality = 256 - (quality * 256);
        }
	}

}
