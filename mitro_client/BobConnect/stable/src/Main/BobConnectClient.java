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

import LZ77.LZ77Compressor;
import LZ77.LZ77Decompressor;
import Ros.RosWrapper;

import javax.imageio.ImageIO;
import javax.imageio.ImageWriteParam;
import javax.swing.*;
import javax.swing.filechooser.*;
import javax.swing.filechooser.FileFilter;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.image.BufferedImage;
import java.awt.image.ImageFilter;
import java.io.*;
import java.math.BigInteger;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketAddress;
import java.security.MessageDigest;
import javax.swing.filechooser.FileFilter;

public class BobConnectClient {
    private static JFrame frame;
    private static ImagePanel panel;

    private static ImageStreamEncoder ve; //Encodes images captured from camera

    private static CameraProxy cam; //Captures images from camera

    //private static AudioCapturePlayer audio; //Capture and playback audio

    private static BufferedImage owncam=null; //Buffers image from own camera
    private static BufferedImage stillIMG=null; //Buffers image from own camera
    //private static BufferedImage prev=null; //Buffers previous image

    private static long lastframe=0;



    public static void main(String[] args) throws Exception{

        //Parse command line parameters
        String host,dev=null;
        int port=0;
        if (args.length==0){



            dev="/dev/video0";  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEFAULT DEVICE



            host="swarmlab-srv02.unimaas.nl";
            port=50000;
        }
        else{
            if (args.length==1){
                dev=args[0];
                host="swarmlab-srv02.unimaas.nl";
                port=50000;
            }
            else{
                try{
                    port=Integer.parseInt(args[1]);
                }
                catch(Exception ex){
                    System.out.println("\""+args[1]+"\" does not seem to be a valid port number");
                    printInstructionsAndExit();
                }
                host=args[0];
                if (args.length>2&&!args[2].trim().isEmpty())
                    dev=args[2];
            }
        }

        JFrame loginframe=new JFrame("MITRO");
        loginframe.setSize(640,480);
        loginframe.setResizable(false);
        LoginGUI gui=new LoginGUI();
        loginframe.add(gui);
        loginframe.setVisible(true);
        loginframe.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        //Socket to connect to server

        boolean connected=false;
        boolean loginfail=false;
        while(!connected){
            Socket s1;
            loginfail=false;
            try{
                //If server requires login
                //if (Constants.SERVER_REQUIRES_LOGIN){


                    while(!gui.isDone()){
                        try{Thread.sleep(30);}catch(Exception ex){}
                    }
                    String username=gui.getUsername();
                    String password=gui.getPassword();
                    gui.setStatus("Connecting...");
                    //Read user data from stdin
                    /*Console c = System.console();
                    c.printf("Username: ");
                    String username=c.readLine();
                    c.printf("Password: ");
                    String password=new String(c.readPassword());*/

                    //Open connection to server
                    System.out.println("Connecting video to "+host+":"+port);

                    SocketAddress sockaddr = new InetSocketAddress(host, port);
                    s1 = new Socket();
                    s1.connect(sockaddr, 1000);

                    if (!s1.isConnected()){
                        gui.setStatus("Connection failed.");
                    }
                    else{

                        gui.setStatus("Connected...");

                        s1.setTrafficClass(254);
                        //Create Streamwriter
                        BufferedWriter writer=new BufferedWriter(new OutputStreamWriter(s1.getOutputStream()));

                        //Send username
                        writer.write(username+"\n");
                        writer.flush();

                        //Create Streamreader
                        BufferedReader reader=new BufferedReader(new InputStreamReader(s1.getInputStream()));

                        //Receive salt from server
                        String salt=reader.readLine();

                        //Create SHA-256 Hasher
                        MessageDigest md=MessageDigest.getInstance("SHA-256");

                        //Create hash of salted password
                        String sha256=new BigInteger(1,md.digest((password+salt).getBytes("UTF8"))).toString(16);

                        //Send hash to server
                        writer.write(sha256+"\n");
                        writer.flush();

                        String s=reader.readLine();
                        if (s.compareTo("1")!=0){
                            try{
                                reader.close();
                                writer.close();
                                s1.close();
                            }
                            catch(Exception ex){}
                            loginfail=true;
                            gui.setStatus("Login failed.");
                            gui.reset();
                            System.out.println("Login failed.");
                        }
                    //}
                    //else{

                        //Open connection to server
                     //   System.out.println("Connecting video to "+host+":"+port);
                     //   s1 =new Socket(host, port);
                    //}

                    //Connect audio stream


                    sockaddr = new InetSocketAddress(host, port+1);
                    Socket s2 = new Socket();
                    if (!loginfail){
                        System.out.println("Connecting audio to "+host+":"+(port+1));
                        s2.connect(sockaddr, 1000);
                    }

                    if (loginfail || !s2.isConnected()){
                        gui.setStatus("login failed.");
                    }
                    else{

                        //Socket s2=new Socket(host,port+1);
                        s2.setTrafficClass(254);

                        //Create audio module
                        if (Constants.AUDIO_COMPRESS_STREAMS){
                            //audio=new AudioCapturePlayer(new BufferedInputStream(s2.getInputStream(),4*Constants.AUDIO_BUFFER_SIZE),new BufferedOutputStream(s2.getOutputStream(),Constants.AUDIO_BUFFER_SIZE));
                        }
                        else{
                            //audio=new AudioCapturePlayer(new BufferedInputStream(s2.getInputStream(),20000),new BufferedOutputStream(s2.getOutputStream(),Constants.AUDIO_BUFFER_SIZE));
                        }

                        //Open data connection
                        System.out.println("Connecting data to "+host+":"+(port+2));
                        final Socket s3=new Socket(host,port+2);
                        s3.setTrafficClass(254);
                        //Create Stream decoder for incoming video stream
                        ImageStreamDecoder dec=new ImageStreamDecoder(new BufferedInputStream(s1.getInputStream()),new ImageStreamDecoderListener() {
                            boolean lock=false;

                            //We received a new image
                            public void nextImage(BufferedImage img) {

                                //If we aren't finished with the previous frame's1 processing we drop the frame
                                if (lock)return;
                                lock=true;
                                try{
                                    //Get Graphics object of incoming image
                                    Graphics2D g=(Graphics2D)img.getGraphics();

                                    //Draw own camera image into it if available
                                    //if (owncam!=null)g.drawImage(owncam,20,3*img.getHeight()/4,img.getWidth()/4,img.getHeight()/4,null);

                                    //Display image
                                    panel.setImage(img,owncam);

                                }catch(Exception ex){}
                                lock=false;
                            }
                        });

                        //If we have a camera device we create a camera proxy to capture images
                         if (dev!=null){
                            cam=new CameraProxy(Constants.VIDEO_CAPTURE_DRIVER,dev,Constants.CAPUTRE_WIDTH,Constants.CAPUTRE_HEIGHT,new CameraProxyListener() {
                                boolean lock=false;

                                BufferedImage prev=null; //Buffers previous image

                                long lastframe=0;
                                public void nextImage(BufferedImage img) {
                                    if (stillIMG!=null)img=ImageTools.copyImage(stillIMG);


                                    //If we haven't processed the previous frame yet, we drop the new one
                                    //Buffer own camera image
                                    try{
                                        owncam=img;

                                        panel.setOwnImage(owncam);
                                    }
                                    catch(Exception ex){
                                        ex.printStackTrace();
                                    }

                                    if (lock)return;
                                    lock=true;
                                    try{


                                        //Estimate difference to previous image
                                        double diff=ImageTools.getDifference(img,prev);

                                        //Get current time stamp
                                        long now=System.currentTimeMillis();

                                        if (now-lastframe>50){

                                            //If the difference exceeds a minimum threshold
                                            if (diff>Constants.MIN_DIFFERENCE){

                                                //If the change is in the image is low
                                                //if (diff<=Constants.MAX_LOW_CHANGE_DIFFERENCE){

                                                    //Encode with low change setting
                                                //	ve.encode(img, Constants.LOW_CHANGE_QUALITY, ImageWriteParam.MODE_EXPLICIT);
                                                //}
                                                //else{

                                                    //Encode with high change setting
                                                    ve.encode(img, Constants.HIGH_CHANGE_QUALITY, ImageWriteParam.MODE_EXPLICIT);
                                                //}

                                                //Buffer previous image
                                                prev=img;

                                                //Set timestamp of last sent frame
                                                lastframe=now;
                                            }

                                            //If we haven't sent a frame in 500ms we do so now
                                            else if (now-lastframe>100){

                                                //Encode with low framerate setting
                                                ve.encode(img, Constants.LOW_FRAMERATE_QUALITY, ImageWriteParam.MODE_EXPLICIT);

                                                prev=img;

                                                lastframe=now;
                                            }
                                        }

                                        //If the write has failed, probably the pipe is broken
                                        if (ve.hasWriteFailed()){

                                            //If the camera proxy is initialized we stop it...
                                            if (cam!=null)cam.stop();

                                            //If audio is running we sto it...
                                            //if (audio!=null)audio.stop();

                                            //...and exit
                                            System.exit(0);
                                        }
                                    }catch(Exception ex){ex.printStackTrace();}
                                    lock=false;
                                }
                            });
                         }

                        gui.setStatus("Opening streams...");
                        loginframe.setVisible(false);
                        //Initialize frame to display video
                        frame=new JFrame("MITRO");
                        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
                        frame.setLayout(new GridLayout(1,1));

                        //RosWrapper ros=new RosWrapper("~/ros/rosWrapper/bin/rosWrapper");

                        final OutputStreamWriter dataout=new OutputStreamWriter(s3.getOutputStream());

                        final boolean[] downkeys=new boolean[5];
                        KeyListener kl=new KeyListener(){
                            public void keyTyped(KeyEvent e) {
                                try{
                                    switch(e.getKeyCode()){
                                        case KeyEvent.VK_1:
                                            panel.setCamMode(0);
                                            dataout.write(Constants.DATA_MSG_CAM+"0\n");
                                            dataout.flush();
                                            break;
                                        case KeyEvent.VK_2:
                                            panel.setCamMode(1);
                                            dataout.write(Constants.DATA_MSG_CAM+"1\n");
                                            dataout.flush();
                                            break;
                                        case KeyEvent.VK_3:
                                            panel.setCamMode(2);
                                            dataout.write(Constants.DATA_MSG_CAM+"2\n");
                                            dataout.flush();
                                            break;
                                        case KeyEvent.VK_4:
                                            panel.setCamMode(3);
                                            dataout.write(Constants.DATA_MSG_CAM+"3\n");
                                            dataout.flush();
                                            break;
                                    }
                                }
                                catch(Exception ex){
                                    ex.printStackTrace();
                                }
                            }
                            public void keyPressed(KeyEvent e) {

                                try{
                                    switch(e.getKeyCode()){
                                        case KeyEvent.VK_1:
                                            dataout.write(Constants.DATA_MSG_CAM+"0\n");
                                            dataout.flush();
                                            break;
                                        case KeyEvent.VK_2:
                                            dataout.write(Constants.DATA_MSG_CAM+"1\n");
                                            dataout.flush();
                                            break;
                                        case KeyEvent.VK_3:
                                            dataout.write(Constants.DATA_MSG_CAM+"2\n");
                                            dataout.flush();
                                            break;
                                        case KeyEvent.VK_4:
                                            dataout.write(Constants.DATA_MSG_CAM+"3\n");
                                            dataout.flush();
                                            break;
                                    }
                                }
                                catch(Exception ex){
                                    ex.printStackTrace();
                                }
                                switch(e.getKeyCode()){
                                    case KeyEvent.VK_UP:
                                        downkeys[0]=true;
                                        break;
                                    case KeyEvent.VK_LEFT:
                                        downkeys[1]=true;
                                        break;
                                    case KeyEvent.VK_RIGHT:
                                        downkeys[2]=true;
                                        break;
                                    case KeyEvent.VK_DOWN:
                                        downkeys[3]=true;
                                        break;
                                    case KeyEvent.VK_SPACE:
                                        downkeys[4]=true;
                                        break;
                                }

                            }
                            public void keyReleased(KeyEvent e) {
                               switch(e.getKeyCode()){
                                    case KeyEvent.VK_UP:
                                        downkeys[0]=false;
                                        break;
                                    case KeyEvent.VK_LEFT:
                                        downkeys[1]=false;
                                        break;
                                    case KeyEvent.VK_RIGHT:
                                        downkeys[2]=false;
                                        break;
                                    case KeyEvent.VK_DOWN:
                                        downkeys[3]=false;
                                        break;
                                    case KeyEvent.VK_SPACE:
                                        downkeys[4]=false;
                                        break;
                                }
                            }
                        };

                        new Thread(){
                            public void run(){
                                boolean stopped=true;
                                while(true){
                                    try{
                                        if (downkeys[4]){
                                            dataout.write(RosWrapper.CMDS_STOP+"\n");
                                            dataout.flush();
                                            stopped=false;
                                        }
                                        else if (downkeys[3]){
                                            dataout.write(RosWrapper.CMDS_BACK+"\n");
                                            dataout.flush();
                                            stopped=false;
                                        }
                                        else if (downkeys[0]){
                                            if (downkeys[1]){
                                                dataout.write(RosWrapper.CMDS_FORWARD_LEFT+"\n");
                                                dataout.flush();
                                                stopped=false;
                                            }
                                            else if (downkeys[2]){
                                                dataout.write(RosWrapper.CMDS_FORWARD_RIGHT+"\n");
                                                dataout.flush();
                                                stopped=false;
                                            }
                                            else{
                                                dataout.write(RosWrapper.CMDS_FORWARD+"\n");
                                                dataout.flush();
                                                stopped=false;
                                            }
                                        }
                                        else if (downkeys[1]){
                                            dataout.write(RosWrapper.CMDS_LEFT+"\n");
                                            dataout.flush();
                                            stopped=false;
                                        }
                                        else if (downkeys[2]){
                                            dataout.write(RosWrapper.CMDS_RIGHT+"\n");
                                            dataout.flush();
                                            stopped=false;
                                        }
                                        else if (!stopped){
                                            dataout.write(RosWrapper.CMDS_STOP+"\n");
                                            dataout.flush();
                                            stopped=true;
                                        }
                                        Thread.sleep(Constants.DATA_MSG_INTERVAL);

                                    }catch(Exception ex){
                                        ex.printStackTrace();
                                    }

                                }
                            }
                        }.start();

                        panel=new ImagePanel(true,kl);
                        panel.setPanelListener(new ImagePanelListener(){
                            public void sendCommand(String command) {
                                try{
                                    dataout.write(command+"\n");
                                    dataout.flush();
                                }catch(Exception ex){
                                    ex.printStackTrace();
                                }
                            }

                            public void loadImage() {
                                if (stillIMG!=null){
                                    stillIMG=null;
                                }
                                else{
                                    final JFileChooser fc=new JFileChooser();
                                    fc.setFileFilter(new FileFilter(){
                                        public boolean accept(File f) {
                                            if (f.isHidden())return false;
                                            if (f.isDirectory())return true;
                                            int i=f.getName().lastIndexOf('.');
                                            if (i>=0){
                                                String fn=f.getName().substring(i+1).toLowerCase();
                                                if (fn.compareTo("png")==0||
                                                    fn.compareTo("jpg")==0||
                                                    fn.compareTo("bmp")==0||
                                                    fn.compareTo("gif")==0||
                                                    fn.compareTo("jpeg")==0)
                                                    return true;
                                            }
                                            return false;
                                        }

                                        public String getDescription() {
                                            return "Images";
                                        }
                                    });
                                    int res=fc.showOpenDialog(frame);
                                    if (res==JFileChooser.APPROVE_OPTION){
                                        new Thread(){
                                            public void run(){
                                                try{
                                                    BufferedImage img= ImageIO.read(fc.getSelectedFile());
                                                    img=ImageTools.scaleImageToRectangle(img,Constants.CAPUTRE_WIDTH,9*Constants.CAPUTRE_WIDTH/16,BufferedImage.TYPE_INT_RGB);
                                                    BufferedImage img2=new BufferedImage(Constants.CAPUTRE_WIDTH,9*Constants.CAPUTRE_WIDTH/16,BufferedImage.TYPE_INT_RGB);
                                                    Graphics g=img2.getGraphics();
                                                    g.drawImage(img,(img2.getWidth()-img.getWidth())/2,(img2.getHeight()-img.getHeight())/2,null);
                                                    stillIMG=img2;
                                                }
                                                catch(Exception ex){
                                                    JOptionPane.showMessageDialog(frame,"Error loading image");
                                                }
                                            }
                                        }.start();
                                    }
                                }
                            }

                            public void flipImage() {
                                cam.flipImage();
                            }
                        }
                        );

                        frame.add(panel);
                        frame.addKeyListener(kl);
                        frame.setSize(320,240);
                        frame.setVisible(true);


                        new Thread(){
                            public void run(){
                                try{
                                    BufferedReader in=new BufferedReader(new InputStreamReader(s3.getInputStream()));
                                    //System.out.println("DATA MSG INIT");
                                    String s=in.readLine();
                                    while(s!=null){
                                        //System.out.println("DATA MSG: "+s);
                                        if (s.startsWith(Constants.DATA_MSG_WIFI)){
                                            try{
                                                double d=Double.parseDouble(s.substring(Constants.DATA_MSG_WIFI.length()));
                                                panel.setSigWifi(d);
                                            }
                                            catch(Exception ex){
                                                ex.printStackTrace();
                                            }
                                        }
                                        else if (s.startsWith(Constants.DATA_MSG_BATT)){
                                            try{
                                                double d=Double.parseDouble(s.substring(Constants.DATA_MSG_BATT.length()));
                                                panel.setSigBatt(d);
                                            }
                                            catch(Exception ex){
                                                ex.printStackTrace();
                                            }
                                        }
                                        else if (s.startsWith(Constants.DATA_MSG_SPEED)){
                                            try{
                                                double d=Double.parseDouble(s.substring(Constants.DATA_MSG_SPEED.length()));
                                                panel.setSigSpeed(d/2);
                                            }
                                            catch(Exception ex){
                                                ex.printStackTrace();
                                            }
                                        }
                                        else if (s.startsWith(Constants.DATA_MSG_DELAY)){ //Set connection latency
                                            try{
                                                long d=Long.parseLong(s.substring(Constants.DATA_MSG_DELAY.length()));
                                                double dt=Math.max(0,Math.min(1,1 - (2*Math.atan(d/300.0))/Math.PI));
                                                //System.out.println(d+" => "+dt);
                                                panel.setSigDelay(dt);
                                            }
                                            catch(Exception ex){
                                                ex.printStackTrace();
                                            }
                                        }
                                        else if (s.startsWith(Constants.DATA_MSG_PING)){ //Return ping
                                            try{
                                                dataout.write(s+"\n");
                                                dataout.flush();
                                            }
                                            catch(Exception ex){
                                                ex.printStackTrace();
                                            }
                                        }
                                        else if (s.startsWith(Constants.DATA_MSG_CAM)){ //Set camera mode
                                            try{
                                                int camMode=Integer.parseInt(s.substring(Constants.DATA_MSG_CAM.length()));
                                                panel.setCamMode(camMode);
                                            }
                                            catch(Exception ex){
                                                ex.printStackTrace();
                                            }
                                        }


                                        s=in.readLine();
                                    }
                                }
                                catch(Exception ex){
                                    ex.printStackTrace();
                                }
                            }
                        }.start();

                        //Start decoder
                        dec.start();

                        //If camera proxy initialized
                        if (cam!=null){

                            //Create output stream from socket connection
                            BufferedOutputStream out=new BufferedOutputStream(s1.getOutputStream());

                            //Create stream encoder and pipe into socket stream
                            ve=new ImageStreamEncoder(out);

                            //Start camera proxy
                            cam.start();
                        }

                        //Start audio module
                        //audio.start();

                        connected=s1.isConnected()&&s2.isConnected()&&s3.isConnected();
                    }
                }
            }
            catch(Exception ex){
                gui.reset();
                if (!loginfail)gui.setStatus("Connection failed.");
                loginframe.setVisible(true);
                if (frame!=null)frame.setVisible(false);
                ex.printStackTrace();
            }
        }
    }

    public static void printInstructionsAndExit(){
        System.out.println(" Usage: BobConnectClient <server> <port> <camdev>");
        System.out.println(" Example: BobConnectClient 192.168.1.5 19914 /dev/video0");
        System.exit(0);
    }

}
