# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
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
#  * Neither the name of Fraunhofer nor the names of its
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


from python_qt_binding import QtCore, QtGui

import time
import math

import roslib
import roslib.message
import rospy
import threading

class EchoDialog(QtGui.QDialog):
  
  MESSAGE_HZ_LIMIT = 10
  MAX_DISPLAY_MSGS = 25
  STATISTIC_QUEUE_LEN = 5000

  '''
  This dialog shows the output of a topic.
  '''
  
  finished_signal = QtCore.Signal(str)
  '''
  finished_signal has as parameter the name of the topic and is emitted, if this
  dialog was closed.
  '''
  
  msg_signal = QtCore.Signal(str)
  '''
  msg_signal is a signal, which is emitted, if a new message was received.
  '''
  
  def __init__(self, topic, type, show_only_rate=False, masteruri=None, parent=None):
    '''
    Creates an input dialog.
    @param topic: the name of the topic
    @type topic: C{str}
    @param type: the type of the topic
    @type type: C{str}
    @raise Exception: if no topic class was found for the given type
    '''
    QtGui.QDialog.__init__(self, parent=parent)
    masteruri_str = '' if masteruri is None else ''.join([' [', str(masteruri), ']'])
    self.setObjectName(' - '.join(['EchoDialog', topic, masteruri_str]))
    self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
    self.setWindowFlags(QtCore.Qt.Window)
    self.setWindowTitle(''.join(['Echo of ' if not show_only_rate else 'Hz of ', topic, masteruri_str]))
    self.resize(728,512)
    self.verticalLayout = QtGui.QVBoxLayout(self)
    self.verticalLayout.setObjectName("verticalLayout")
    self.verticalLayout.setContentsMargins(1, 1, 1, 1)
    self.mIcon = QtGui.QIcon(":/icons/crystal_clear_prop_run_echo.png")
    self.setWindowIcon(self.mIcon)
        
    self.topic = topic
    self.show_only_rate = show_only_rate
    self.lock = threading.Lock()
    self.last_printed_tn = 0
    self.msg_t0 = -1.
    self.msg_tn = 0
    self.times =[]
        
    self.message_count = 0

    self._rate_message = ''

    self._last_received_ts = 0
    self.receiving_hz = self.MESSAGE_HZ_LIMIT

    self.field_filter_fn = None

    options = QtGui.QWidget(self)
    if not show_only_rate:
      hLayout = QtGui.QHBoxLayout(options)
      hLayout.setContentsMargins(1, 1, 1, 1)
      self.no_str_checkbox = no_str_checkbox = QtGui.QCheckBox('Hide strings')
      no_str_checkbox.toggled.connect(self.on_no_str_checkbox_toggled)
      hLayout.addWidget(no_str_checkbox)
      self.no_arr_checkbox = no_arr_checkbox = QtGui.QCheckBox('Hide arrays')
      no_arr_checkbox.toggled.connect(self.on_no_arr_checkbox_toggled)
      hLayout.addWidget(no_arr_checkbox)
      # add spacer
      spacerItem = QtGui.QSpacerItem(515, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
      hLayout.addItem(spacerItem)
      # add combobox for displaying frequency of messages
      self.combobox_displ_hz = QtGui.QComboBox(self)
      self.combobox_displ_hz.addItems([str(self.MESSAGE_HZ_LIMIT), '0.1', '1', '50', '100'])
      self.combobox_displ_hz.activated[str].connect(self.on_combobox_hz_activated)
      self.combobox_displ_hz.setEditable(True)
      hLayout.addWidget(self.combobox_displ_hz)
      displ_hz_label = QtGui.QLabel('Hz', self)
      hLayout.addWidget(displ_hz_label)
      # add combobox for count of displayed messages
      self.combobox_msgs_count = QtGui.QComboBox(self)
      self.combobox_msgs_count.addItems([str(self.MAX_DISPLAY_MSGS), '50', '100'])
      self.combobox_msgs_count.activated[str].connect(self.on_combobox_count_activated)
      self.combobox_msgs_count.setEditable(True)
      hLayout.addWidget(self.combobox_msgs_count)
      displ_count_label = QtGui.QLabel('displayed count', self)
      hLayout.addWidget(displ_count_label)
      # add topic control button for unsubscribe and subscribe
      self.topic_control_button = QtGui.QToolButton(self)
      self.topic_control_button.setText('stop')
      self.topic_control_button.setIcon(QtGui.QIcon(':/icons/deleket_deviantart_stop.png'))
      self.topic_control_button.clicked.connect(self.on_topic_control_btn_clicked)
      hLayout.addWidget(self.topic_control_button)
      # add clear button
      clearButton = QtGui.QToolButton(self)
      clearButton.setText('clear')
      clearButton.clicked.connect(self.on_clear_btn_clicked)
      hLayout.addWidget(clearButton)
      self.verticalLayout.addWidget(options)
    
    self.display = QtGui.QTextEdit(self)
    self.display.setReadOnly(True)
    self.verticalLayout.addWidget(self.display);
    self.display.document().setMaximumBlockCount(500)
    self.max_displayed_msgs = self.MAX_DISPLAY_MSGS
    self._blocks_in_msg = None

    self.status_label = QtGui.QLabel('0 messages', self)
    self.verticalLayout.addWidget(self.status_label)

    # subscribe to the topic
    self.__msg_class = roslib.message.get_message_class(type)
    if not self.__msg_class:
      raise Exception("Cannot load message class for [%s]. Are your messages built?"%type)
    self.sub = rospy.Subscriber(self.topic, self.__msg_class, self._msg_handle)
    self.msg_signal.connect(self._append_message)
    
    self.print_hz_timer = QtCore.QTimer()
    self.print_hz_timer.timeout.connect(self._on_calc_hz)
    self.print_hz_timer.start(1000)

#    print "======== create", self.objectName()
#
#  def __del__(self):
#    print "******* destroy", self.objectName()

#  def hideEvent(self, event):
#    self.close()

  def closeEvent (self, event):
    if not self.sub is None:
      self.sub.unregister()
      del self.sub
    self.finished_signal.emit(self.topic)
    if self.parent() is None:
      QtGui.QApplication.quit()
#    else:
#      self.setParent(None)
  
  def create_field_filter(self, echo_nostr, echo_noarr):
    def field_filter(val):
      try:
        fields = val.__slots__
        field_types = val._slot_types
        for f, t in zip(val.__slots__, val._slot_types):
          if echo_noarr and '[' in t:
            continue
          elif echo_nostr and 'string' in t:
            continue
          yield f
      except:
        pass
    return field_filter

  def on_no_str_checkbox_toggled(self, state):
    self.field_filter_fn = self.create_field_filter(state, self.no_arr_checkbox.isChecked())

  def on_no_arr_checkbox_toggled(self, state):
    self.field_filter_fn = self.create_field_filter(self.no_str_checkbox.isChecked(), state)

  def on_combobox_hz_activated(self, hz_txt):
    try:
      self.receiving_hz = int(hz_txt)
    except ValueError:
      try:
        self.receiving_hz = float(hz_txt)
      except ValueError:
        self.combobox_displ_hz.setEditText(str(self.receiving_hz))

  def on_combobox_count_activated(self, count_txt):
    try:
      self.max_displayed_msgs = int(count_txt)
      self._blocks_in_msg = None
    except ValueError:
      self.combobox_msgs_count.setEditText(str(self.max_displayed_msgs))

  def on_clear_btn_clicked(self):
    self.display.clear()
    with self.lock:
      self.message_count = 0
      del self.times[:]

  def on_topic_control_btn_clicked(self):
    if self.sub is None:
      self.sub = rospy.Subscriber(self.topic, self.__msg_class, self._msg_handle)
      self.topic_control_button.setText('stop')
      self.topic_control_button.setIcon(QtGui.QIcon(':/icons/deleket_deviantart_stop.png'))
    else:
      self.sub.unregister()
      self.sub = None
      self.topic_control_button.setText('play')
      self.topic_control_button.setIcon(QtGui.QIcon(':/icons/deleket_deviantart_play.png'))

  def _msg_handle(self, data):
    self.msg_signal.emit(roslib.message.strify_message(data, field_filter=self.field_filter_fn))

  def _append_message(self, msg):
    '''
    Adds a label to the dialog's layout and shows the given text.
    @param msg: the text to add to the dialog
    @type msg: C{str}
    '''
    current_time = time.time()
    with self.lock:
      # time reset
      if self.msg_t0 < 0 or self.msg_t0 > current_time:
        self.msg_t0 = current_time
        self.msg_tn = current_time
        self.times = []
      else:
        self.times.append(current_time - self.msg_tn)
        self.msg_tn = current_time

      #only keep statistics for the last 5000 messages so as not to run out of memory
      if len(self.times) > self.STATISTIC_QUEUE_LEN:
        self.times.pop(0)

    self.message_count += 1
    # skip messages, if they are received often then MESSAGE_HZ_LIMIT 
    if self._last_received_ts != 0:
      if current_time - self._last_received_ts < 1.0 / self.receiving_hz:
        return 
    self._last_received_ts = current_time

    if not self.show_only_rate:
      txt = ''.join(['<pre style="background-color:#FFFCCC; font-family:Fixedsys,Courier,monospace; padding:10px;">------------------------------\n\n', msg,'</pre>'])
      # set the count of the displayed messages on receiving the first message
      if self._blocks_in_msg is None:
        td = QtGui.QTextDocument(txt)
        self._blocks_in_msg = td.blockCount()
        self.display.document().setMaximumBlockCount(self._blocks_in_msg*self.max_displayed_msgs)
      self.display.append(txt)
    self._print_status()

  def _on_calc_hz(self):
    if rospy.is_shutdown():
      self.close()
      return
    if self.msg_tn == self.last_printed_tn:
      self._rate_message = 'no new messages'
      return
    with self.lock:
      # the code from ROS rostopic
      n = len(self.times)
      if n == 0:
        return
      mean = sum(self.times) / n
      rate = 1./mean if mean > 0. else 0

      #std dev
      std_dev = math.sqrt(sum((x - mean)**2 for x in self.times) /n)
      # min and max
      max_delta = max(self.times)
      min_delta = min(self.times)

      self.last_printed_tn = self.msg_tn
      self._rate_message = "average rate: %.3f\tmin: %.3fs   max: %.3fs   std dev: %.5fs   window: %s"%(rate, min_delta, max_delta, std_dev, n+1)
      self._print_status()
      if self.show_only_rate:
        self.display.append(self._rate_message)
#        status_label = QtGui.QLabel(self._rate_message, self)
#        self.contentLayout.addWidget(status_label)

  def _print_status(self):
    status_text = ' '.join([str(self.message_count), 'messages', ', ' if self._rate_message else '', self._rate_message])
    self.status_label.setText(status_text)
