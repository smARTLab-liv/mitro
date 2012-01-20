package Ros;

import java.io.*;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

/**
 * Created by IntelliJ IDEA.
 * User: homer
 * Date: Oct 13, 2011
 * Time: 3:51:42 PM
 * To change this template use File | Settings | File Templates.
 */
public class RosWrapper {
    private OutputStreamWriter out;
    private InputStream in;
    private SendThread thread;
    private RosWrapperListener list;

    public RosWrapper(String path,RosWrapperListener l) throws Exception{
        this.list=l;
        Process p=Runtime.getRuntime().exec(path);
        out=new OutputStreamWriter(p.getOutputStream());
        in=p.getInputStream();
        new Thread(){
            public void run(){
                try{
                    InputStreamReader read=new InputStreamReader(in);
                    char[] input=new char[1000];
                    int x=0;
                    int y=1;//read.read(input,x,1);
                    while(y>0){
                        while(y>0&&(x==0||input[x-1]!=':')&&x<1000){
                            //System.out.println(input[x]+" "+(byte)input[x])
                            y=read.read(input,x,1);
                            x++;
                        }
                        String out="";
                        for (int a=0;a<x-1;a++){
                            //System.out.print(input[a]);
                            out+=input[a];
                        }
                        //System.out.println(out);
                        if (list!=null)list.receivedMessage(out);
                        x=0;
                    }
                        /*try{

                            String s=in.readLine();
                            while(s!=null){
                    System.out.println("fun "+s);
                                if (list!=null)list.receivedMessage(s);
                                s=in.readLine();
                            }*/
                }
                catch(Exception ex){
                    //ex.printStackTrace();
                }
            }
        }.start();

    }

    public static final int CMD_FORWARD=0;
    public static final int CMD_FORWARD_LEFT=1;
    public static final int CMD_FORWARD_RIGHT=2;
    public static final int CMD_LEFT=3;
    public static final int CMD_RIGHT=4;
    public static final int CMD_BACK=5;
    public static final int CMD_STOP=6;

    public static final String CMDS_FORWARD="FW";
    public static final String CMDS_FORWARD_LEFT="FL";
    public static final String CMDS_FORWARD_RIGHT="FR";
    public static final String CMDS_LEFT="LE";
    public static final String CMDS_RIGHT="RI";
    public static final String CMDS_BACK="BK";
    public static final String CMDS_STOP="ST";



    public void sendCommand(String cmd) throws Exception{
        if (thread!=null)thread.stopIt();
        thread=new SendThread(cmd);
        thread.start();
            //out.write(cmd+"\n");
            //out.flush();
    }


    class SendThread extends Thread{
        private String cmd;
        private boolean stop;
        public SendThread(String cmd){
            stop=false;
            this.cmd=cmd;
        }
        public void run(){
            try{
                for (int x=0;x<4;x++){
                    out.write(cmd+"\n");
                    out.flush();
                    Thread.sleep(100);
                    if (stop)return;
                }
            }
            catch(Exception ex){
                //ex.printStackTrace();
            }
        }
        public void stopIt(){
            stop=true;
        }
    }

    /*public void sendCommand(int cmd) throws Exception{
        switch(cmd){
            case CMD_FORWARD:
                out.write(CMDS_FORWARD+"\n");
                out.flush();
                break;
            case CMD_FORWARD_LEFT:
                out.write(CMDS_FORWARD_LEFT+"\n");
                out.flush();
                break;
            case CMD_FORWARD_RIGHT:
                out.write(CMDS_FORWARD_RIGHT+"\n");
                out.flush();
                break;
            case CMD_LEFT:
                out.write(CMDS_LEFT+"\n");
                out.flush();
                break;
            case CMD_RIGHT:
                out.write(CMDS_RIGHT+"\n");
                out.flush();
                break;
            case CMD_BACK:
                out.write(CMDS_BACK+"\n");
                out.flush();
                break;
            case CMD_STOP:
                out.write(CMDS_STOP+"\n");
                out.flush();
                break;
        }
    }*/

	public static void main(String[] args) throws Exception{
		new RosWrapper("/home/swarmlab/ros/rosWrapper/bin/rosWrapper",null);
		while(true);
	}

}
