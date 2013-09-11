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

import com.xuggle.xuggler.*;
import com.xuggle.xuggler.video.ConverterFactory;
import com.xuggle.xuggler.video.IConverter;

import java.awt.image.BufferedImage;
import java.io.IOException;

public class CameraProxy {
    private String driver,device;
    private int iWidth,iHeight;
    private IContainer container;
    private int videoStreamId;
    private IStreamCoder videoCoder;
    private IVideoResampler resampler;
    private WebcamThread thread;
    private CameraProxyListener list;
    private int captureWidth,captureHeight;
    private boolean flipImage;

    public CameraProxy(String driver, String device, int iWidth, int iHeight, CameraProxyListener list) throws RuntimeException, IllegalArgumentException, IOException {

		this.driver=driver;
		this.device=device;
		this.iWidth=iWidth;
		this.iHeight=iHeight;
        this.list=list;
        this.flipImage=false;

        captureWidth=iWidth;
        captureHeight=9*captureWidth/16;


		container = IContainer.make();

        IContainerParameters params = IContainerParameters.make();

        params.setTimeBase(IRational.make(Constants.CAPTURE_FRAMERATE, 1));

        params.setVideoWidth(iWidth);
        params.setVideoHeight(iHeight);

        container.setParameters(params);

        IContainerFormat format = IContainerFormat.make();
        if (format.setInputFormat(driver) < 0)
          throw new IllegalArgumentException("couldn't open webcam device: " + driver);

        int retval = container.open(device, IContainer.Type.READ, format);
        if (retval < 0)
        {
          IError error = IError.make(retval);
          throw new IllegalArgumentException("could not open file: " + device + "; Error: " + error.getDescription());
        }

        // query how many streams the call to open found
        int numStreams = container.getNumStreams();

        System.out.println(numStreams+" Stream(s) found");


        videoStreamId = -1;
        videoCoder = null;
        for(int i = 0; i < numStreams; i++)
        {
          // Find the stream object
          IStream stream = container.getStream(i);
          // Get the pre-configured decoder that can decode this stream;
          IStreamCoder coder = stream.getStreamCoder();



          if (coder.getCodecType() == ICodec.Type.CODEC_TYPE_VIDEO)
          {
            videoStreamId = i;
            videoCoder = coder;
            break;
          }
        }

        if (videoStreamId == -1)
          throw new RuntimeException("could not find video stream in container: "+device);

        /*
         * Now we have found the video stream in this file.  Let's open up our decoder so it can
         * do work.
         */
        if (videoCoder.open() < 0)
          throw new RuntimeException("could not open video decoder for container: "+device);


        if (videoCoder.getPixelType() != IPixelFormat.Type.BGR24)
        {
          // if this stream is not in BGR24, we're going to need to
          // convert it.  The VideoResampler does that for us.
          resampler = IVideoResampler.make(videoCoder.getWidth(), videoCoder.getHeight(), IPixelFormat.Type.BGR24,
              videoCoder.getWidth(), videoCoder.getHeight(), videoCoder.getPixelType());
          if (resampler == null)
            throw new RuntimeException("could not create color space resampler for: " + device);
        }
	}

    public void start(){
        thread=new WebcamThread();
        thread.start();
    }

    public void stop(){
        thread.stopIt();
    }

	public class WebcamThread extends Thread{
        //private BufferedImage cropBuffer;
        private boolean stop=false;

        /*public WebcamThread() {
            int captureHeight=9*Constants.CAPUTRE_WIDTH/16;
            cropBuffer=new BufferedImage(Constants.CAPUTRE_WIDTH,captureHeight,BufferedImage.TYPE_3BYTE_BGR);
        } */

        public void stopIt(){
            stop=true;
        }
        public void run(){
        	try{
	            /*
	             * Now, we start walking through the container looking at each packet.
	             */
	            IPacket packet = IPacket.make();
	            stop=false;
	            IConverter conv= ConverterFactory.createConverter(ConverterFactory.XUGGLER_BGR_24, IPixelFormat.Type.BGR24, iWidth, iHeight);

	            while(!stop&&container.readNextPacket(packet) >= 0)
	            {
	              if (packet.getStreamIndex() == videoStreamId)
	              {
	                IVideoPicture picture = IVideoPicture.make(videoCoder.getPixelType(),videoCoder.getWidth(), videoCoder.getHeight());
	                int offset = 0;
	                while(offset < packet.getSize())
	                {
	                  /*
	                   * Now, we decode the video, checking for any errors.
	                   *
	                   */
	                  int bytesDecoded = videoCoder.decodeVideo(picture, packet, offset);
	                  if (bytesDecoded < 0)
	                    throw new RuntimeException("got error decoding video");
	                  offset += bytesDecoded;

	                  /*
	                   * Some decoders will consume data in a packet, but will not be able to construct
	                   * a full video picture yet.  Therefore you should always check if you
	                   * got a complete picture from the decoder
	                   */
	                  if (picture.isComplete())
	                  {
	                    IVideoPicture newPic = picture;
	                    /*
	                     * If the resampler is not null, that means we didn't get the video in BGR24 format and
	                     * need to convert it into BGR24 format.
	                     */
	                    if (resampler != null)
	                    {
	                      // we must resample
	                      newPic = IVideoPicture.make(resampler.getOutputPixelFormat(), picture.getWidth(), picture.getHeight());
	                      if (resampler.resample(newPic, picture) < 0)
	                        throw new RuntimeException("could not resample video");
	                    }
	                    if (newPic.getPixelType() != IPixelFormat.Type.BGR24)
	                      throw new RuntimeException("could not decode video as BGR 24 bit data");

	                    // Convert the BGR24 to an Java buffered image
	                    final BufferedImage javaImage = conv.toImage(newPic);

                        new Thread(){
                            public void run(){

                                //BufferedImage cropBuffer=new BufferedImage(Constants.CAPUTRE_WIDTH,captureHeight,BufferedImage.TYPE_3BYTE_BGR);
                                list.nextImage(ImageTools.cropImage(javaImage,captureWidth,captureHeight,flipImage));
                            }
                        }.start();


	                    try{
	                        Thread.sleep(5);
	                    }
	                    catch (Exception ex){}
	                  }
	                }
	              }
	            }
	        }catch(Exception ex){
	        	ex.printStackTrace();
	        }
            if (videoCoder != null){
                videoCoder.close();
                videoCoder = null;
            }
            if (container !=null){
                container.close();
                container = null;
            }
        }
    }

    public String getDriver() {
        return driver;
    }

    public String getDevice() {
        return device;
    }

    public int getiWidth() {
        return iWidth;
    }

    public int getiHeight() {
        return iHeight;
    }

    public void flipImage() {
        flipImage=!flipImage;
    }

}
