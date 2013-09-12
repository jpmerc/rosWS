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

import threading
from python_qt_binding import QtCore
from python_qt_binding import QtGui

import rospy

import node_manager_fkie as nm
from detailed_msg_box import WarningMessageBox, DetailedError

class InteractionNeededError(Exception):
  ''' 
  request: AuthenticationRequest
  '''  
  def __init__(self, request, method, args):
    Exception.__init__(self)
    self.method = method
    self.request = request
    self.args = args
  
  def __str__(self):
    return "InteractionNeededError"
  

class ProgressQueue(QtCore.QObject):
  
  def __init__(self, progress_frame, progress_bar, progress_cancel_button):
    QtCore.QObject.__init__(self)
    self.__progress_queue = []
    self._progress_frame = progress_frame
    self._progress_bar = progress_bar
    self._progress_cancel_button = progress_cancel_button
    progress_frame.setVisible(False)
    progress_cancel_button.clicked.connect(self._on_progress_canceled)
  
  def stop(self):
    try:
      val = self._progress_bar.value()
      if val < len(self.__progress_queue):
        print "  Shutdown progress queue..."
        thread = self.__progress_queue[val]
        self.__progress_queue = []
        thread.join(3)
        print "  Progress queue is off!"
    except:
      pass


  def add2queue(self, id, descr, target=None, args=()):
    pt = ProgressThread(id, descr, target, args)
    pt.finished_signal.connect(self._progress_thread_finished)
    pt.error_signal.connect(self._progress_thread_error)
    pt.request_interact_signal.connect(self._on_request_interact)
    self.__progress_queue.append(pt)
    self._progress_bar.setMaximum(len(self.__progress_queue))

  def start(self):
    if not self._progress_frame.isVisible() and self.__progress_queue:
      self._progress_frame.setVisible(True)
      self._progress_bar.setToolTip(self.__progress_queue[0].descr)
      dscr_len = self._progress_bar.size().width()/10
      self._progress_bar.setFormat(''.join(['%v/%m - ', self.__progress_queue[0].descr[0:dscr_len]]))
      self._progress_bar.setValue(0)
      self.__progress_queue[0].start()

  def count(self):
    return len(self.__progress_queue)

  def _progress_thread_finished(self, id):
    try:
      #print "PG finished", id
      val = self._progress_bar.value()
      th = self.__progress_queue[val+1]
      self._progress_bar.setToolTip(th.descr)
      dscr_len = self._progress_bar.size().width()/10
      self._progress_bar.setFormat(''.join(['%v/%m - ', th.descr[0:dscr_len]]))
      self.__progress_queue[val+1].start()
      self._progress_bar.setValue(val+1)
      #'print "PG finished ok", id
    except:
      #'print "PG finished err", id
      for thread in self.__progress_queue:
        thread.join(1)
      self._progress_frame.setVisible(False)
      #'print "PG finished delete all..."
      self.__progress_queue = []
      #'print "PG finished delete all ok"

  def _progress_thread_error(self, id, title, msg, detailed_msg):
    btns = (QtGui.QMessageBox.Ignore)
    if len(self.__progress_queue) > 1:
      btns = (QtGui.QMessageBox.Ignore|QtGui.QMessageBox.Abort)
    res = WarningMessageBox(QtGui.QMessageBox.Warning, title, msg, detailed_msg,
                            buttons=btns ).exec_()
    if res == QtGui.QMessageBox.Abort:
      self.__progress_queue = []
      self._progress_frame.setVisible(False)
    else:
      self._progress_thread_finished(id)

  def _on_progress_canceled(self):
    try:
#      self.__progress_queue[self._progress_bar.value()].wait()
      self.__progress_queue = []
      self._progress_frame.setVisible(False)
    except:
      import traceback
      print traceback.format_exc()

  def _on_request_interact(self, id, descr, req):
    if isinstance(req.request, nm.AuthenticationRequest):
      res, user, pw = nm.ssh()._requestPW(req.request.user, req.request.host)
      if not res:
        self._on_progress_canceled()
        return
      pt = ProgressThread(id, descr, req.method, (req.args+(user, pw)))
      pt.finished_signal.connect(self._progress_thread_finished)
      pt.error_signal.connect(self._progress_thread_error)
      pt.request_interact_signal.connect(self._on_request_interact)
      pt.start()
    elif isinstance(req.request, nm.ScreenSelectionRequest):
      from select_dialog import SelectDialog
      items = SelectDialog.getValue('Show screen', req.request.choices.keys(), False)
      if not items:
        self._progress_thread_finished(id)
        return
      res = [req.request.choices[i] for i in items]
      pt = ProgressThread(id, descr, req.method, (req.args+(res,)))
      pt.finished_signal.connect(self._progress_thread_finished)
      pt.error_signal.connect(self._progress_thread_error)
      pt.request_interact_signal.connect(self._on_request_interact)
      pt.start()
    elif isinstance(req.request, nm.BinarySelectionRequest):
      from select_dialog import SelectDialog
      items = SelectDialog.getValue('Multiple executables', req.request.choices, True)
      if not items:
        self._progress_thread_finished(id)
        return
      res = items[0]
      pt = ProgressThread(id, descr, req.method, (req.args+(res,)))
      pt.finished_signal.connect(self._progress_thread_finished)
      pt.error_signal.connect(self._progress_thread_error)
      pt.request_interact_signal.connect(self._on_request_interact)
      pt.start()



class ProgressThread(QtCore.QObject, threading.Thread):
  '''
  A thread to execute a method in a thread.
  '''
  finished_signal = QtCore.Signal(str)
  '''
  @ivar: finished_signal is a signal, which is emitted, if the thread is finished.
  '''

  error_signal = QtCore.Signal(str, str, str, str)
  '''
  @ivar: error_signal is a signal (id, title, error message, detailed error message), which is emitted, 
  if an error while run of the thread was occurred.
  '''

  request_interact_signal = QtCore.Signal(str, str, InteractionNeededError)

  def __init__(self, id, descr='', target=None, args=()):
    QtCore.QObject.__init__(self)
    threading.Thread.__init__(self)
    self._id = id
    self.descr = descr
    self._target = target
    self._args = args
    self.setDaemon(True)

  def run(self):
    '''
    '''
    try:
      if not self._target is None:
        #'print "PG call "
        #'print "  .. ", self._target
        #print "  -- args:", self._args
        self._target(*self._args)
        #print "PG call finished"
        self.finished_signal.emit(self._id)
      else:
        self.error_signal.emit(self._id, 'No target specified')
    except InteractionNeededError as e:
      self.request_interact_signal.emit(self._id, self.descr, e)
    except DetailedError as e:
      self.error_signal.emit(self._id, e.title, e.value, e.detailed_text)
    except:
      import traceback
#      print traceback.print_exc()
      formatted_lines = traceback.format_exc().splitlines()
      last_line = formatted_lines[-1]
      index = 1
      while not last_line and len(formatted_lines) > index:
        index += 1
        last_line = formatted_lines[-index]
      rospy.logwarn("%s failed:\n\t%s", str(self.descr), last_line)
      self.error_signal.emit(self._id, 'Progress Job Error', str(self.descr)+" failed:\n"+last_line, traceback.format_exc())
