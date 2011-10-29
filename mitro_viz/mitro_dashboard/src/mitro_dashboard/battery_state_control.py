# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('mitro_dashboard')

import wx

import math

from os import path

def non_zero(value):
  if value < 0.00001 and value > -0.00001:
    return 0.00001
  return value
  

class BatteryStateControl(wx.Window):
  def __init__(self, parent, id, icons_path):
    wx.Window.__init__(self, parent, id, wx.DefaultPosition, wx.Size(60, 32))
    self.SetBackgroundColour(parent.GetBackgroundColour())

#    self._power_consumption = 0.0
    self._pct = 0
    self._voltage = 0
#    self._cap = 0
#    self._char_cap = 0
#    self._time_remaining = 0.0
#    self._ac_present = 0
    
    self._left_bitmap = wx.Bitmap(path.join(icons_path, "battery-minus.png"), wx.BITMAP_TYPE_PNG)
    self._right_bitmap = wx.Bitmap(path.join(icons_path, "battery-plus.png"), wx.BITMAP_TYPE_PNG)
    self._plug_bitmap = wx.Bitmap(path.join(icons_path, "battery-charging.png"), wx.BITMAP_TYPE_PNG)
    self._background_bitmap = wx.Bitmap(path.join(icons_path, "battery-background.png"), wx.BITMAP_TYPE_PNG)
    self._green = wx.Bitmap(path.join(icons_path, "battery-green-bar.png"), wx.BITMAP_TYPE_PNG)
    self._yellow = wx.Bitmap(path.join(icons_path, "battery-yellow-bar.png"), wx.BITMAP_TYPE_PNG)
    self._red = wx.Bitmap(path.join(icons_path, "battery-red-bar.png"), wx.BITMAP_TYPE_PNG)
    
    self.SetSize(wx.Size(self._left_bitmap.GetWidth() + self._right_bitmap.GetWidth() + self._background_bitmap.GetWidth(), 32))
    
    self._start_x = self._left_bitmap.GetWidth()
    self._end_x = self.GetSize().x - self._right_bitmap.GetWidth()
    self._width = self._end_x - self._start_x
    
    self._plugged_in = False
    
    self.Bind(wx.EVT_PAINT, self.on_paint)

  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)

    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()
 
    w = self.GetSize().GetWidth()
    h = self._left_bitmap.GetHeight()
    
    color_bitmap = None
    if (self._pct > 50):
      color_bitmap = self._green
    elif (self._pct > 30):
      color_bitmap = self._yellow
    else:
      color_bitmap = self._red

    dc.DrawBitmap(self._background_bitmap, self._start_x, 0, True)
    
    color_image = color_bitmap.ConvertToImage()
    scaled_color_image = color_image.Rescale(round(self._width * self._pct/100.0), color_bitmap.GetHeight())
    color_bitmap = wx.BitmapFromImage(scaled_color_image)
    dc.DrawBitmap(color_bitmap, self._start_x, 0, True)
    dc.DrawBitmap(self._left_bitmap, 0, 0, True)
    dc.DrawBitmap(self._right_bitmap, self._end_x, 0, True)
    
    if (self._plugged_in):
      dc.DrawBitmap(self._plug_bitmap, 
                    (self._start_x + self._width) / 2.0 - (self._plug_bitmap.GetWidth() / 2.0), 
                    self.GetSize().GetHeight() / 2.0 - (self._plug_bitmap.GetHeight() / 2.0))
  

  def set_voltage(self, voltage):
    self._voltage = voltage
  
  def set_percent(self, percent):
    self._pct = percent
    self.SetToolTip(wx.ToolTip("Battery: %.0f%% (%.2fV)"%(self._pct, self._voltage)))
    self.Refresh()

  def set_charging(self, charging):
    self._plugged_in = charging
    self.Refresh()

    
  def set_stale(self):
    self._plugged_in = 0
    self._pct = 0
    self._time_remaining = 0.0
    self._power_consumption = 0
    self.SetToolTip(wx.ToolTip("Battery: Stale"))
    
    self.Refresh()
