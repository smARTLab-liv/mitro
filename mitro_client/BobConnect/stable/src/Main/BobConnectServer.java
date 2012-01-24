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

import LZ77.LZ77Compressor;
import LZ77.LZ77Decompressor;
import Ros.*;


import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.math.BigInteger;
import java.net.ServerSocket;
import java.net.Socket;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Random;

import javax.imageio.ImageWriteParam;
import javax.swing.*;

public class BobConnectServer {
	private ImageStreamEncoder ve; //Encoder for captured images

    private CameraProxy cam,cam2; //Proxies for cameras

    //private static AudioCapturePlayer audio; //Capture and playback audio

    private BufferedImage secondCam; //Buffer for second camera's image

    private JFrame frame; //Frame to display client image
    private ImagePanel panel; //Panel to display client image

    private boolean done=false; //Sets when thread is done

    private IwconfigWraper iwc; //Wrapper for iwconfig, reads signal level

    private Userdb userdb; //User database

    private static final boolean DEBUG=false;

    private String lastping=null;
    private long lastpingtime=0;
    private int camMode=0;


	public BobConnectServer(String driver, String device,String device2, String wlandev, int iWidth, int iHeight, ServerSocket socket1, ServerSocket socket2, ServerSocket socket3, Userdb userdb) throws RuntimeException, IllegalArgumentException, IOException{

        //This thread is not done (yet)
        done=false;

        //Start listening on socket
        System.out.println("["+now()+"] Listening on port "+socket1.getLocalPort());
        final Socket s1=socket1.accept();

        s1.setTrafficClass(254);
        //Link user database
        this.userdb=userdb;

        //If userdatabase set, we need a login from the client
        if (userdb!=null && Constants.SERVER_REQUIRES_LOGIN){

            //Create reader for socket
            BufferedReader reader=new BufferedReader(new InputStreamReader(s1.getInputStream()));

            //Read username
            String username=reader.readLine();

            //Create writer for socket
            BufferedWriter writer=new BufferedWriter(new OutputStreamWriter(s1.getOutputStream()));

            //Create random salt
            byte[] saltb=new byte[16];
            new Random().nextBytes(saltb);
            String salt=new BigInteger(saltb).toString(16);

            //Send salt to client
            writer.write(salt+"\n");
            writer.flush();

            //Read salted password hash from client
            String password=reader.readLine();

            //If login invalid
            if (!userdb.checkCredentials(username,password,salt)){

                //Close connection
                s1.close();

                //Thread is done
                done=true;

                //Stop
                return;
            }

            System.out.println("["+now()+"] "+username+" logged in");
        }

        System.out.println("["+now()+"] Video streaming through port "+socket1.getLocalPort());

        //Accept audio connection
        System.out.println("["+now()+"] Listening on port "+socket2.getLocalPort());
        final Socket s2=socket2.accept();

        s2.setTrafficClass(254);

        //Create audio module
        /*if (Constants.AUDIO_COMPRESS_STREAMS){
            audio=new AudioCapturePlayer(new BufferedInputStream(s2.getInputStream(),4*Constants.AUDIO_BUFFER_SIZE),new BufferedOutputStream(s2.getOutputStream(),Constants.AUDIO_BUFFER_SIZE));
        }
        else{
            audio=new AudioCapturePlayer(new BufferedInputStream(s2.getInputStream(),20000),new BufferedOutputStream(s2.getOutputStream(),Constants.AUDIO_BUFFER_SIZE));
        } */



        System.out.println("["+now()+"] Audio streaming through port "+socket2.getLocalPort());

        //Accept data connection
        System.out.println("["+now()+"] Listening on port "+socket3.getLocalPort());
        final Socket s3=socket3.accept();

        s3.setTrafficClass(254);
        System.out.println("["+now()+"] Data streaming through port "+socket3.getLocalPort());

        final OutputStreamWriter dataWriter=new OutputStreamWriter(s3.getOutputStream());
        

        new Thread(){
            public void run(){
                try{
                    while(s3.isConnected()&&!s3.isClosed()){
                        lastping=Constants.DATA_MSG_PING+System.currentTimeMillis();
                        lastpingtime=System.currentTimeMillis();
                        dataWriter.write(lastping+"\n");
                        dataWriter.flush();
                        int x=0;
                        Thread.sleep(100);
                        while(lastping!=null&&x<15){
                            Thread.sleep(100);
                            x++;
                        }
                    }
                }catch(Exception ex){
                    ex.printStackTrace();
                }

            }
        }.start();

        if (!DEBUG){
            try{
                final RosWrapper ros=new RosWrapper("/home/swarmlab/ros/mitro_apps/mitro_telepresence/bin/tele_wrapper", new RosWrapperListener(){
                    public void receivedMessage(String msg) {
                        try{
                            dataWriter.write(msg+"\n");
                            dataWriter.flush();
                        }
                        catch(Exception ex){
                            ex.printStackTrace();
                        }
                    }
                });

                new Thread(){
                    public void run(){

                        try{
                            BufferedReader in=new BufferedReader(new InputStreamReader(s3.getInputStream()));
                            String s=in.readLine();
                            while (s!=null&&s3.isConnected()&&!s3.isClosed()){
                                if (s.startsWith(Constants.DATA_MSG_PING)){
                                    if (lastping!=null&&s.equals(lastping)){
                                        lastping=null;
                                        long delay=System.currentTimeMillis()-lastpingtime;
                                        dataWriter.write(Constants.DATA_MSG_DELAY+delay+"\n");
                                        dataWriter.flush();
                                        if (delay>1000){
                                            System.out.println("Connection problems");
                                        }
                                    }
                                }
                                else if (s.startsWith(Constants.DATA_MSG_CAM)){
                                    try{
                                        camMode=Integer.parseInt(s.substring(Constants.DATA_MSG_CAM.length()));
                                        //System.out.println("CAMMODE "+camMode);
                                        dataWriter.write(s+"\n");
                                        dataWriter.flush();
                                        //FUCK THIS SHIT
                                    }
                                    catch(Exception ex){
                                        ex.printStackTrace();
                                    }
                                }
                                else{
                                    ros.sendCommand(s);
                                }
                                s=in.readLine();
                            }
                        }
                        catch(Exception ex){
                            ex.printStackTrace();
                            //System.exit(0);
                        }
                    }
                }.start();

            }
            catch(Exception ex){
                ex.printStackTrace();
                System.exit(0);
            }
        }
        else{
            new Thread(){
                public void run(){
                    try{
                        BufferedReader in=new BufferedReader(new InputStreamReader(s3.getInputStream()));
                        String s=in.readLine();
                        while (s!=null&&s3.isConnected()&&!s3.isClosed()){
                            if (s.startsWith(Constants.DATA_MSG_PING)){
                                if (lastping!=null&&s.equals(lastping)){
                                    long delay=System.currentTimeMillis()-lastpingtime;
                                    dataWriter.write(Constants.DATA_MSG_DELAY+delay+"\n");
                                    dataWriter.flush();
                                    lastping=null;
                                    if (delay>1000){
                                        System.out.println("Connection problems");
                                    }
                                }
                            }
                            else if (s.startsWith(Constants.DATA_MSG_CAM)){
                                try{
                                    camMode=Integer.parseInt(s.substring(Constants.DATA_MSG_CAM.length()));
                                    //System.out.println("CAMMODE "+camMode);
                                    dataWriter.write(s+"\n");
                                    dataWriter.flush();

                                }
                                catch(Exception ex){
                                    ex.printStackTrace();
                                }
                            }
                            else{
                                System.out.println("ROS COMMAND: "+s);
                            }
                            s=in.readLine();
                        }
                    }
                    catch(Exception ex){
                        ex.printStackTrace();
                        //System.exit(0);
                    }
                }
            }.start();

            //Initialize iwconfig wrapper to read signal level
            iwc=new IwconfigWraper(wlandev);

            //Start iwconfig wrapper
            iwc.start();

            new Thread(){
                public void run(){

                    Random rnd=new Random();
                    while(s3.isConnected()&&!s3.isClosed()){
                        try{
                            Thread.sleep(1000);
                            dataWriter.write(Constants.DATA_MSG_BATT+rnd.nextDouble()+"\n");
                            dataWriter.flush();
                            dataWriter.write(Constants.DATA_MSG_SPEED+rnd.nextDouble()+"\n");
                            dataWriter.flush();
                            dataWriter.write(Constants.DATA_MSG_WIFI+iwc.getLink()+"\n");
                            dataWriter.flush();
                            //System.out.println("DATA MSG: ");
                        }
                        catch(Exception ex){
                            //ex.printStackTrace();
                            //System.exit(0);
                        }
                    }
                }
            }.start();

        }



        //Initialize decoder for client video stream
        ImageStreamDecoder dec=new ImageStreamDecoder(new BufferedInputStream(s1.getInputStream()),new ImageStreamDecoderListener() {
            boolean lock=false;
            public void nextImage(BufferedImage img) {

                //If we are still processing the previous image, drop this one
                if (lock)return;
                lock=true;
                try{
                    //Display image
                    panel.setImage(img);
                }catch(Exception ex){}
                lock=false;
            }
        });

        //Initialize camera proxy
        cam=new CameraProxy(driver,device,iWidth,iHeight,new CameraProxyListener() {

            boolean lock=false; //Lock for frame dropping

            BufferedImage prev=null; //Buffer for previous image

            long lastframe=0; //timestamp of last frame

            public void nextImage(BufferedImage img) {

                //If we are still processing the previous image, drop this one
                if (lock)return;
                lock=true;
                try{
                    switch(camMode){
                        case 0:
                            //Get graphics handle of image
                            Graphics2D g=(Graphics2D)img.getGraphics();
                            //If second camera image is present, draw into image
                            if (secondCam!=null)g.drawImage(secondCam,2*img.getWidth()/3,0,img.getWidth()/3,img.getHeight()/3,null);
                            break;
                        case 1:
                            if (secondCam!=null){
                                //Get graphics handle of image
                                BufferedImage img2=new BufferedImage(img.getWidth(),img.getHeight(),img.getType());
                                g=(Graphics2D)img2.getGraphics();
                                g.drawImage(secondCam,0,0,null);
                                //If second camera image is present, draw into image
                                g.drawImage(img,2*img.getWidth()/3,0,img.getWidth()/3,img.getHeight()/3,null);
                                img=img2;
                            }
                            break;
                        case 3:
                            if (secondCam!=null){
                                img=secondCam;
                            }
                            break;
                    }

                    //Draw status bars

                    //Battery
                    /*g.setColor(Color.WHITE);
                    g.drawRect(img.getWidth()-15,5,10,img.getHeight()-10);
                    g.setColor(Color.GREEN);
                    double bat=0.5;
                    int bath=(int)Math.round((img.getHeight()-10)*bat);
                    if (bat<0.4)
                        g.setColor(Color.RED);
                    else if (bat<0.6)
                        g.setColor(Color.YELLOW);
                    else
                        g.setColor(Color.GREEN);
                    g.fillRect(5,img.getHeight()-5-bath,10,bath);

                    //Link
                    g.setColor(Color.WHITE);
                    g.drawRect(5,5,10,img.getHeight()-10);
                    double link=iwc.getLink();
                    if (link<0.4)
                        g.setColor(Color.RED);
                    else if (link<0.6)
                        g.setColor(Color.YELLOW);
                    else
                        g.setColor(Color.GREEN);
                    int linkh=(int)Math.round((img.getHeight()-10)*link);
                    g.fillRect(img.getWidth()-15,img.getHeight()-5-linkh,10,linkh);
                    g.setColor(Color.WHITE);
                    g.drawRect(img.getWidth()-15,5,10,img.getHeight()-10);*/

                    //Estimate difference between images
                    double diff=ImageTools.getDifference(img,prev);

                    //Current timestamp
                    long now=System.currentTimeMillis();

                    //If minimum difference threshold exceeded
                    if (diff>Constants.MIN_DIFFERENCE){

                        //If low change in image
                        //if (diff<Constants.MAX_LOW_CHANGE_DIFFERENCE){

                            //Encode with low change setting
                        //    ve.encode(img, Constants.LOW_CHANGE_QUALITY, ImageWriteParam.MODE_EXPLICIT);
                        //}
                        //else{

                            //Encode with high change setting
                            ve.encode(img, Constants.HIGH_CHANGE_QUALITY, ImageWriteParam.MODE_EXPLICIT);
                        //}

                        //Buffer previous image
                        prev=img;

                        //Save timestamp as last processes frame
                        lastframe=now;
                    }

                    //If we haven't processed a frame in 500ms do so now
                    else if (now-lastframe>100){

                        //Encode with low framerate setting
                        ve.encode(img, Constants.LOW_FRAMERATE_QUALITY, ImageWriteParam.MODE_EXPLICIT);

                        //Buffer previous image
                        prev=img;

                        //Save timestamp as last processes frame
                        lastframe=now;

                    }

                    //If write failed probably the pipe broke
                    if (ve.hasWriteFailed()){

                        //Stop running camera proxies
                        if (cam!=null)cam.stop();
                        if (cam2!=null)cam2.stop();


                        //if (audio!=null)audio.stop();



                        //Hide video frame
                        frame.setVisible(false);

                        //Close all sockets
                        try{s1.close();}catch(Exception ex){}
                        try{s2.close();}catch(Exception ex){}
                        try{s3.close();}catch(Exception ex){}
                        System.out.println("["+now()+"] Disconnected ");

                        //Mark thread as done
                        done=true;
                    }
                }catch(Exception ex){ex.printStackTrace();}
                lock=false;
            }
        });

        //If second camera active
        if (device2!=null){

            //Create camera proxy
            cam2=new CameraProxy(driver,device2,iWidth,iHeight,new CameraProxyListener() {

                boolean lock=false; //Lock for framedropping

                BufferedImage prev=null; //Buffer for previous image

                //timestamp of last frame
                long lastframe=0;

                public void nextImage(BufferedImage img) {
                    //If we are still processing the previous image, drop this one
                    if (lock)return;
                    lock=true;
                    try{

                        //Buffer image to be rendered into video stream
                        secondCam=img;

                    }catch(Exception ex){ex.printStackTrace();}
                    lock=false;
                }
            });
        }

        //Create outputstream for socket
        BufferedOutputStream out=new BufferedOutputStream(s1.getOutputStream());

        //Initialize stream encoder
        ve=new ImageStreamEncoder(out);

        //Construct frame
        frame=new JFrame("Webcam");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(new GridLayout(1,1));
        panel=new ImagePanel();
        frame.add(panel);
	    frame.setSize(320,240);

        //Set frame to fullscreen
        //frame.setUndecorated(true);
        Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
        frame.setBounds(0,0,screenSize.width, screenSize.height);
        //frame.setResizable(false);
        frame.setAlwaysOnTop(true);

        //Show frame
        frame.setVisible(true);

        //Start decoder
        dec.start();

        //Start camera proxies
        cam.start();
        if (cam2!=null)cam2.start();

        //Start audio module
        /*try{
            audio.start();
        }
        catch(Exception ex){
            ex.printStackTrace();
        } */

	}

    public boolean isDone(){
        return done;
    }

    public static void main(String[] args)throws Exception{

        //Parse command line parameters
        int port=0;
        String dev0=null,dev1=null;
        if (args.length==0){
            printInstructionsAndExit();
        }
        try{
            port=Integer.parseInt(args[0]);
        }
        catch(Exception ex){
            System.out.println("\""+args[0]+"\" does not seem to be a valid port number");
            printInstructionsAndExit();
        }
        if (args.length>1&&!args[1].trim().isEmpty())dev0=args[1];
        if (args.length>2&&!args[2].trim().isEmpty())dev1=args[2];

        //Load user database
        Userdb userdb=new Userdb(new File("./user.db"));

        //Create serversocket for video
        ServerSocket socket1=new ServerSocket(port);

        //Create serversocket for audio
        ServerSocket socket2=new ServerSocket(port+1);

        //Create serversocket for data
        ServerSocket socket3=new ServerSocket(port+2);

        //Keep it going
        while (true){
            try{

                //Start webcamserver thread
                BobConnectServer srv=new BobConnectServer(Constants.VIDEO_CAPTURE_DRIVER, dev0, dev1,Constants.WLAN_DEVICE, Constants.CAPUTRE_WIDTH, Constants.CAPUTRE_HEIGHT, socket1,socket2,socket3,userdb);

                //Sleep while thread is working
                while(!srv.isDone()){
                    Thread.sleep(100);
                }
            }
            catch(Exception ex){
                ex.printStackTrace();
            }
        }
    }

    public static void printInstructionsAndExit(){
        System.out.println(" Usage: BobConnectServer <port> <cam0dev> [cam1dev]");
        System.out.println(" Example: BobConnectServer 19914 /dev/video0 /dev/video1");
        System.exit(0);
    }

    private static final String DATE_FORMAT_NOW = "yyyy-MM-dd HH:mm:ss";

    public static String now() {
        Calendar cal = Calendar.getInstance();
        SimpleDateFormat sdf = new SimpleDateFormat(DATE_FORMAT_NOW);
        return sdf.format(cal.getTime());
    }
}
