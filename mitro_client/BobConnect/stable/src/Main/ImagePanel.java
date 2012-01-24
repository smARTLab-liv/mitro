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

import Ros.RosWrapper;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.concurrent.locks.ReentrantLock;

public class ImagePanel extends JComponent {
    private BufferedImage in,owncam;
    private BufferedImage img=null;
    private BufferedImage panel,panelsized=null;
    private BufferedImage[] arrows, arrowssized, camIcons, camIconsSized;
    private boolean lock=false;
    private boolean client;
    private long lastmousemove=0;
    private int[] dimGaugeWifi,dimGaugeBatt,dimGaugeSpeed,dimControls,dimPanel,dimOwncam,dimArrows,dimCam;
    private ImagePanelListener list;

    private double sigWifi=0,sigBatt=0,sigSpeed=0,sigDelay=0;
    private int camMode=0;

    private static final int[] DIM_GAUGE_WIFI={510,375,472,410,548,410};
    private static final int[] DIM_GAUGE_BATT={738,375,700,410,776,410};
    private static final int[] DIM_GAUGE_SPEED={1306,375,1268,410,1344,410};
    private static final int[] DIM_CONTROLS={959,376,20,53};
    private static final int[] DIM_PANEL={173};
    private static final int[] DIM_OWN_CAM={6,10,382,289};
    private static final int[] DIM_ARROWS={906,323,108,108};//867,357,184,184};
    private static final int[] DIM_CAM={1073,326,108,108};//1156,365,176,176};

    private ReentrantLock imageLock;

    private int direction=-1;

    private boolean mouseDown=false;
    private double mouseAngle=0;

    public ImagePanel(){
        this(false,null);
    }
    public ImagePanel(boolean client,KeyListener kl){
        this.client=client;
        this.imageLock=new ReentrantLock();

        if (client){
            try{
                panel= ImageIO.read(new File("panel.png"));
                arrows=new BufferedImage[6];
                arrows[0]=ImageIO.read(new File("le.png"));
                arrows[1]=ImageIO.read(new File("lf.png"));
                arrows[2]=ImageIO.read(new File("fr.png"));
                arrows[3]=ImageIO.read(new File("rf.png"));
                arrows[4]=ImageIO.read(new File("ri.png"));
                arrows[5]=ImageIO.read(new File("bk.png"));
                camIcons=new BufferedImage[4];
                BufferedImage tmp=ImageIO.read(new File("camIcons.png"));
                camIcons[0]=tmp.getSubimage(0,0,54,54);
                camIcons[1]=tmp.getSubimage(54,0,54,54);
                camIcons[2]=tmp.getSubimage(0,54,54,54);
                camIcons[3]=tmp.getSubimage(54,54,54,54);
            }
            catch(Exception ex){
                ex.printStackTrace();
                System.exit(0);
            }

            this.addMouseMotionListener(new HIDListener());
            this.addMouseListener(new HIDListener());
            this.addKeyListener(kl);
        }
    }

    public void setPanelListener(ImagePanelListener l) {
        this.list = l;

        new Thread(){
            private boolean stopped=true;
            public void run(){
                int olddirection=-1;
                while(true){
                    try{
                        if (mouseDown){
                            if (mouseAngle<-2.7 || mouseAngle>=2.32){
                                if (list!=null)list.sendCommand(RosWrapper.CMDS_LEFT);
                                direction=0;
                            }
                            else if (mouseAngle>=-2.7 && mouseAngle<-1.95){
                                if (list!=null)list.sendCommand(RosWrapper.CMDS_FORWARD_LEFT);
                                direction=1;
                            }
                            else if (mouseAngle>=-1.95 && mouseAngle<-1.2){
                                if (list!=null)list.sendCommand(RosWrapper.CMDS_FORWARD);
                                direction=2;
                            }
                            else if (mouseAngle>=-1.2 && mouseAngle<-0.445){
                                if (list!=null)list.sendCommand(RosWrapper.CMDS_FORWARD_RIGHT);
                                direction=3;
                            }
                            else if (mouseAngle>=-0.445 && mouseAngle<0.82){
                                if (list!=null)list.sendCommand(RosWrapper.CMDS_RIGHT);
                                direction=4;
                            }
                            else if (mouseAngle>=0.82 && mouseAngle<2.32){
                                if (list!=null)list.sendCommand(RosWrapper.CMDS_BACK);
                                direction=5;
                            }
                            stopped=false;
                        }
                        else if (!stopped){
                            if (list!=null)list.sendCommand(RosWrapper.CMDS_STOP);
                            stopped=true;
                            direction=-1;
                        }
                        if (direction!=olddirection){
                            olddirection=direction;
                            updateImage();
                        }
                        Thread.sleep(Constants.DATA_MSG_INTERVAL);
                        //System.out.println("ANGLE= "+angle);
                    }
                    catch(Exception ex){
                        ex.printStackTrace();
                    }

                }
            }
        }.start();

    }

    public void setImage(BufferedImage in){
        setImage(in,null);
    }

    private double counter=0;

    public void updateImage(){
        if (imageLock.isLocked())return;
        imageLock.lock();
        //if (lock)return;
        //lock=true;
        try{
            if (!client){
                if (img==null||img.getWidth()!=this.getWidth()||img.getHeight()!=this.getHeight())
                    img=new BufferedImage(this.getWidth(),this.getHeight(),BufferedImage.TYPE_3BYTE_BGR);


                int w=this.getWidth();
                int h=this.getHeight();

                double r1=(double)in.getWidth()/in.getHeight();
                double r2=(double)w/h;

                int ihn=h;
                int iwn=(int)Math.round(ihn*r1);
                if (r1>r2){
                    iwn=w;
                    ihn=(int)Math.round(iwn/r1);
                }

                int ox=(w-iwn)/2;
                int oy=(h-ihn)/2;
                Graphics2D g=(Graphics2D)img.getGraphics();
                g.drawImage(in,ox,oy,iwn,ihn,null);
            }
            else{
                if (img==null||img.getWidth()!=this.getWidth()||img.getHeight()!=this.getHeight()){
                    img=new BufferedImage(this.getWidth(),this.getHeight(),BufferedImage.TYPE_3BYTE_BGR);
                    panelsized=ImageTools.scaleImageToWidth(panel,img.getWidth(),BufferedImage.TYPE_4BYTE_ABGR);
                    double scale=(double)panelsized.getWidth()/panel.getWidth();
                    dimGaugeWifi=scaleArray(DIM_GAUGE_WIFI,scale);
                    dimGaugeBatt=scaleArray(DIM_GAUGE_BATT,scale);
                    dimGaugeSpeed=scaleArray(DIM_GAUGE_SPEED,scale);
                    dimControls=scaleArray(DIM_CONTROLS,scale);
                    dimPanel=scaleArray(DIM_PANEL,scale);
                    dimOwncam=scaleArray(DIM_OWN_CAM,scale);
                    dimArrows=scaleArray(DIM_ARROWS,scale);
                    dimCam=scaleArray(DIM_CAM,scale);
                    arrowssized=new BufferedImage[6];
                    for (int x=0;x<6;x++){
                        arrowssized[x]=ImageTools.scaleImage(arrows[x],scale,BufferedImage.TYPE_3BYTE_BGR);
                    }
                    camIconsSized=new BufferedImage[4];
                    for (int x=0;x<4;x++){
                        camIconsSized[x]=ImageTools.scaleImage(camIcons[x],scale,BufferedImage.TYPE_3BYTE_BGR);
                    }
                }

                int w=this.getWidth();
                int h=this.getHeight()-dimPanel[0];

                Graphics2D g=(Graphics2D)img.getGraphics();

                if (in!=null){
                    double r1=(double)in.getWidth()/in.getHeight();
                    double r2=(double)w/h;

                    int ihn=h;
                    int iwn=(int)Math.round(ihn*r1);
                    if (r1>r2){
                        iwn=w;
                        ihn=(int)Math.round(iwn/r1);
                    }
                    int ox=(w-iwn)/2;
                    int oy=(h-ihn)/2;



                    //g.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                    //                    RenderingHints.VALUE_INTERPOLATION_BICUBIC);

                    g.drawImage(in,ox,oy,iwn,ihn,null);
                }

                g.drawImage(panelsized,0,getHeight()-panelsized.getHeight(),null);

                if (owncam!=null)g.drawImage(owncam,dimOwncam[0],getHeight()-panelsized.getHeight()+dimOwncam[1],dimOwncam[2],dimOwncam[3],null);

                g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                                    RenderingHints.VALUE_ANTIALIAS_ON);


                /*g.setColor(Color.RED);
                g.setStroke(new BasicStroke(1));

                drawGauge(dimGaugeWifi,sigDelay,img.getHeight()-panelsized.getHeight(),g);
                */


                g.setColor(Color.BLACK);
                g.setStroke(new BasicStroke(3));
                drawGauge(dimGaugeWifi,sigWifi,img.getHeight()-panelsized.getHeight(),g);
                drawGauge(dimGaugeBatt,sigBatt,img.getHeight()-panelsized.getHeight(),g);
                drawGauge(dimGaugeSpeed,sigSpeed,img.getHeight()-panelsized.getHeight(),g);

                switch(camMode){
                    case 0:
                        g.drawImage(camIconsSized[camMode],dimCam[0],img.getHeight()-panelsized.getHeight()+dimCam[1],null);
                        break;
                    case 1:
                        g.drawImage(camIconsSized[camMode],dimCam[0]+dimCam[2]/2,img.getHeight()-panelsized.getHeight()+dimCam[1],null);
                        break;
                    case 2:
                        g.drawImage(camIconsSized[camMode],dimCam[0],img.getHeight()-panelsized.getHeight()+dimCam[1]+dimCam[3]/2,null);
                        break;
                    case 3:
                        g.drawImage(camIconsSized[camMode],dimCam[0]+dimCam[2]/2,img.getHeight()-panelsized.getHeight()+dimCam[1]+dimCam[3]/2,null);
                        break;
                }

                if (direction>=0){
                    int ax=dimArrows[0];
                    int ay=getHeight()-panelsized.getHeight()+dimArrows[1];
                    g.drawImage(arrowssized[direction],ax,ay,null);
                }
                
                counter+=0.01;
                if (counter>1)counter=0;

            }

        }
        catch(Exception ex){
            ex.printStackTrace();
        }
        //lock=false;
        imageLock.unlock();
        repaint();
    }

    public void setOwnImage(BufferedImage owncam){
        this.owncam=owncam;
        updateImage();
    }

    public void setImage(BufferedImage in,BufferedImage owncam){
        this.in=in;
        this.owncam=owncam;
        updateImage();
    }

    private static void drawGauge(int[] dimGauge, double value, int yoffset, Graphics g){
        value=Math.min(1,Math.max(value,0));
        double d=Math.sqrt(Math.pow(dimGauge[0]-dimGauge[2],2)+Math.pow(dimGauge[1]-dimGauge[3],2));
        double a1=Math.atan2(dimGauge[3]-dimGauge[1],dimGauge[2]-dimGauge[0]);
        double a2=Math.atan2(dimGauge[5]-dimGauge[1],dimGauge[4]-dimGauge[0]);
        double m=2*Math.PI-(a1-a2);
        double a=a1+value*m;
        int sx=(int)Math.round(dimGauge[0]+Math.cos(a)*5);
        int sy=yoffset+(int)Math.round(dimGauge[1]+Math.sin(a)*5);
        int ex=(int)Math.round(dimGauge[0]+Math.cos(a)*d);
        int ey=yoffset+(int)Math.round(dimGauge[1]+Math.sin(a)*d);
        g.drawLine(sx,sy,ex,ey);

    }

    private static int[] scaleArray(int[] in, double scale){
        int[] out=new int[in.length];
        for (int x=0;x<in.length;x++){
            out[x]=(int)Math.round(in[x]*scale);
        }
        return out;
    }

    public void setSigWifi(double sigWifi) {
        this.sigWifi = sigWifi;
        updateImage();
    }

    public void setSigBatt(double sigBatt) {
        this.sigBatt = sigBatt;
        updateImage();
    }

    public void setSigSpeed(double sigSpeed) {
        this.sigSpeed = sigSpeed;
        updateImage();
    }

    public void setSigDelay(double sigDelay) {
        this.sigDelay = sigDelay;
        updateImage();
    }

    public void setCamMode(int camMode) {
        this.camMode=camMode;
        updateImage();
    }


    class HIDListener implements MouseMotionListener, MouseListener {
        public void mouseDragged(MouseEvent e) {
            int mx=e.getX();
            int my=e.getY()-(getHeight()-panelsized.getHeight());
            double d=Math.sqrt(Math.pow(mx-dimControls[0],2)+Math.pow(my-dimControls[1],2));
            if (d>dimControls[2]&&d<dimControls[3]){
                mouseAngle=Math.atan2(my-dimControls[1],mx-dimControls[0]);
                mouseDown=true;
            }
            else{
                mouseDown=false;
            }
        }

        public void mouseMoved(MouseEvent e) {
            //lastmousemove=System.currentTimeMillis();
        }

        public void mouseClicked(MouseEvent e) {

        }

        public void mousePressed(MouseEvent e) {
            int mx=e.getX();
            int my=e.getY()-(getHeight()-panelsized.getHeight());
            if (mx>=dimCam[0]&&mx<=dimCam[0]+dimCam[2]/2){
                if (my>=dimCam[1]&&my<=dimCam[1]+dimCam[3]/2){
                    list.sendCommand(Constants.DATA_MSG_CAM+"0");
                    camMode=0;
                    repaint();
                }
                else if (my>=dimCam[1]+dimCam[3]/2&&my<=dimCam[1]+dimCam[3]){
                    list.sendCommand(Constants.DATA_MSG_CAM+"2");
                    camMode=2;
                    repaint();
                }
            }
            else if (mx>=dimCam[0]+dimCam[2]/2&&mx<=dimCam[0]+dimCam[2]){
                if (my>=dimCam[1]&&my<=dimCam[1]+dimCam[3]/2){
                    list.sendCommand(Constants.DATA_MSG_CAM+"1");
                    camMode=1;
                    repaint();
                }
                else if (my>=dimCam[1]+dimCam[3]/2&&my<=dimCam[1]+dimCam[3]){
                    list.sendCommand(Constants.DATA_MSG_CAM+"3");
                    camMode=3;
                    repaint();
                }
            }
            else{
                double d=Math.sqrt(Math.pow(mx-dimControls[0],2)+Math.pow(my-dimControls[1],2));
                if (d>dimControls[2]&&d<dimControls[3]){
                    mouseAngle=Math.atan2(my-dimControls[1],mx-dimControls[0]);
                    mouseDown=true;
                }
                else{
                    mouseDown=false;
                }
            }
            
        }

        public void mouseReleased(MouseEvent e) {
            mouseDown=false;
        }

        public void mouseEntered(MouseEvent e) {

        }

        public void mouseExited(MouseEvent e) {

        }
    }



    public void update(Graphics g){
        paint(g);
    }

    public void paint(Graphics g){
        //if (imageLock.isLocked())return;
        imageLock.lock();
        if (img!=null){
            g.drawImage(img,0,0,null);
        }
        imageLock.unlock();
    }

}
