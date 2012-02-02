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

public class Constants {

    //GENERAL
    public static final boolean SERVER_REQUIRES_LOGIN=true;

    //SIGNAL STRENGTH
    public static final String WLAN_DEVICE="wlan0";
    public static final int WLAN_UPDATE_INTERVAL=1000;

    //VIDEO

    public static final int CAPTURE_FRAMERATE=10;
    public static final float LOW_FRAMERATE_QUALITY=0.999f;
    public static final float LOW_CHANGE_QUALITY=0.990f;
    public static final float HIGH_CHANGE_QUALITY=0.999f;
    public static final double MIN_DIFFERENCE=15.0;
    public static final double MAX_LOW_CHANGE_DIFFERENCE=40.0;
    public static final String VIDEO_CAPTURE_DRIVER="video4linux2";
    public static final int CAPUTRE_WIDTH=640;
    public static final int CAPUTRE_HEIGHT=480;

    //AUDIO
    public static final float AUDIO_SAMPLE_RATE=22050;
    public static final int AUDIO_SAMPLE_BITS=8;
    public static final int AUDIO_CHANNELS=1;
    public static final boolean AUDIO_SIGNED=true;
    public static final boolean AUDIO_BIG_ENDIAN=true;
    public static final int AUDIO_BUFFER_SIZE =(int)(AUDIO_SAMPLE_RATE*AUDIO_SAMPLE_BITS*AUDIO_CHANNELS/8)/4; // 1s for good buffering
    public static final int AUDIO_LZ77_STREAM_BITS=8;
    public static final int AUDIO_LZ77_DECOMPRESOR_BUFFER_SIZE=(int)(AUDIO_SAMPLE_RATE*AUDIO_SAMPLE_BITS*AUDIO_CHANNELS/8); //1s for buffering decompressed data
    public static final boolean AUDIO_COMPRESS_STREAMS=false;

    //DATA MESSAGES

    public static final String DATA_MSG_BATT="BAT";
    public static final String DATA_MSG_WIFI="SIG";
    public static final String DATA_MSG_SPEED="SPD";
    public static final String DATA_MSG_CAM="CAM";
    public static final String DATA_MSG_PING="PING";
    public static final String DATA_MSG_DELAY="DEL";

    public static final int DATA_MSG_INTERVAL=50;
    
}
