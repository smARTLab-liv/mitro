/*
Copyright (C) 2011

This file is part of JLZ77
written by Max Bügler
http://www.maxbuegler.eu/

JLZ77 is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option) any
later version.

JLZ77 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

package LZ77;

/**
 * Represents a sequence of bytes to be matched by the LZ77 algorithm
 */
public class LZSequence {

    private byte[] seq; //Bytes
    private int index; //Number of bytes in sequence

    /**
     * Creates a new sequence for a maximum of buffersize bytes
     * @param buffersize maximum number of bytes in sequence
     */
    public LZSequence(int buffersize){
        seq=new byte[buffersize];
        index=0;
    }

    /**
     * Add byte to sequence
     * @param b byte
     */
    public void add(byte b){
        seq[index++]=b;
    }

    /**
     * Get byte from sequence
     * @param x byte index
     * @return byte
     */
    public byte get(int x){
        return seq[x];
    }

    /**
     * Get last byte in sequence (usually the non matching one)
     * @return byte
     */
    public byte getLast(){
        return seq[index-1];
    }

    /**
     * Returns number of bytes in sequence
     * @return number of bytes stored
     */
    public int size(){
        return index;
    }

    /**
     * Checks whether there is a matching for the sequence in window win
     * @param win window
     * @return -1 if no matching, otherwise shift
     */
    public int isInWindow(LZWindow win){
        
        if (win.size()<index)return -1;
        for (int y=index-1;y<win.size();y++){
            for (int x=0;x<index;x++){
                if (win.get(y-x)!=seq[x])
                    break;
                else if (x==index-1){
                    return y;
                }
            }
        }
        return -1;
    }
}
