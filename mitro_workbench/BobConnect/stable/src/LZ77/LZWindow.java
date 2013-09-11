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
package LZ77;

/**
 * Represents the window used for the LZ77 algorithm, so it stores the last wsize bytes added
 */
public class LZWindow {
    private byte[] window; //Bytes
    private int size, offset; //Size and offset of last byte

    /**
     * Creates new window with size wsize
     * @param wsize windowsize
     */
    public LZWindow(int wsize){
        this.window=new byte[wsize];
        this.size=0;
        this.offset=0;
    }

    /**
     * Adds byte to the window (if window full, last byte is discarted)
     * @param b byte
     */
    public void add(byte b){
        if (size<window.length){
            window[size]=b;
            size++;
        }
        else{
            window[offset]=b;
            offset++;
            if (offset>=window.length)offset=0;
        }
    }

    /**
     * Gets byte from window that lies x steps in the past
     * @param x steps into the past
     * @return byte
     */
    public byte get(int x){
        if (size<window.length){
            return window[size-x-1];
        }
        else{
            int p=offset-x-1;
            if (p<0)p+=size;
            return window[p];
        }
    }

    /**
     * Returns number of bytes stored in window
     * @return number of bytes
     */
    public int size(){
        return size;
    }
}
