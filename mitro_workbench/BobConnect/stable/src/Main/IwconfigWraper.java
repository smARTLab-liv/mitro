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

import java.io.BufferedReader;
import java.io.InputStreamReader;

public class IwconfigWraper {

    private static final String LINK_STRING="Link Quality=";

    private boolean stop=false;
    private String dev;
    private IWThread thread;
    private double link=0;

    public IwconfigWraper(String dev){
        this.dev=dev;
    }

    public void start(){
        this.stop=false;
        this.thread=new IWThread();
        this.thread.start();
    }
    public void stop(){
        this.stop=true;
    }

    public double getLink(){
        return link;
    }


    class IWThread extends Thread{
        public void run(){
            while (!stop){
                try{
                    Process p=Runtime.getRuntime().exec("iwconfig "+dev);
                    BufferedReader reader=new BufferedReader(new InputStreamReader(p.getInputStream()));
                    String res=reader.readLine();

                    boolean found=false;
                    while (!found&&res!=null){
                        int i=res.indexOf(LINK_STRING);
                        if (i>=0){

                            res=res.substring(i+LINK_STRING.length());
                            i=res.indexOf(" ");
                            if (i>=0){

                                res=res.substring(0,i);
                                i=res.indexOf("/");
                                try{
                                    int nom=Integer.parseInt(res.substring(0,i));
                                    int den=Integer.parseInt(res.substring(i+1));
                                    link=(double)nom/den;
                                    found=true;
                                }
                                catch(Exception ex){ex.printStackTrace();}


                            }
                        }
                        if (!found)res=reader.readLine();
                    }
                    if (res==null&&!found)link=0;
                    try{
                        p.destroy();
                    }
                    catch(Exception ex){}
                    Thread.sleep(Constants.WLAN_UPDATE_INTERVAL);
                }
                catch(Exception ex){
                    ex.printStackTrace();
                }

            }
        }
    }

    public static void main(String[] args) throws Exception{
        IwconfigWraper w=new IwconfigWraper("wlan0");
        w.start();
        while(true){
            Thread.sleep(100);
        }
    }
}
