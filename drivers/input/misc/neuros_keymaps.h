/*
 *  Copyright(C) 2006-2007 Neuros Technology International LLC. 
 *               <www.neurostechnology.com>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 *  This program is distributed in the hope that, in addition to its 
 *  original purpose to support Neuros hardware, it will be useful 
 *  otherwise, but WITHOUT ANY WARRANTY; without even the implied 
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 ****************************************************************************
 *
 * IR Receiver driver / key mappings file
 *
 * TODO: support 2 different mappings (current keyboard-area mapping and another mapping which uses media keys)
 * TODO: make the 2 mappings switchable via ioctl. Maybe allow remapping at runtime via ioctl.
 *
 */

#define NUM_KEYS 36

static int keymap[NUM_KEYS] = {
    0,                //0
    KEY_KP1,          //1
    KEY_KP2,          //2
    KEY_KP3,          //3
    KEY_KP4,          //4
    KEY_KP5,          //5
    KEY_KP6,          //6
    KEY_KP7,          //7
    KEY_KP8,          //8
    KEY_KP9,          //9
    KEY_KP0,          //10
    0,                //11
    0,                //12
    0,                //13
    0,                //14
    KEY_KPASTERISK,   //15    "*"
    KEY_KPDOT,        //16    "#"  //this was the only way.
    KEY_PAGEUP,       //17    CHAN_UP
    KEY_PAGEDOWN,     //18    CHAN_DN
    KEY_BACKSPACE,    //19    BACK
    KEY_ESC,          //20    HOME 
    KEY_UP,           //21    UP
    KEY_DOWN,         //22    DOWN
    KEY_LEFT,         //23    LEFT
    KEY_RIGHT,        //24    RIGHT
    KEY_ENTER,        //25    ENTER
    KEY_H,            //26    HELP  // no better idea than H
    KEY_TAB,          //27    XIM   // TAB seems just better than X, no other reason
    KEY_U,            //28    UNLABELED_1  //this is a tiny "Unknown" and "Unlabeled" tiny key on remote
    KEY_KPMINUS,      //29    REW
    KEY_KPPLUS,       //30    FFW
    KEY_SPACE,        //31    PLAY  //Most media apps use space for quick pause/play
    KEY_HOME,         //32    PREV
    KEY_END,          //33    NEXT
    KEY_DELETE,       //34    STOP
    KEY_INSERT        //35    RECORD
};

