# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Revision $Id: graph.py 8649 2010-03-12 19:05:53Z kwc $
"""
Master-less rospy node api.

The ronin API is more handle-based than the rospy API. This is in
order to provide better forwards compatibility for ronin as changes to
rospy are made to enable this functionality more gracefully.

Here is an example ronin-based talker:::

    n = ronin.init_node('talker', anonymous=True)
    pub = n.Publisher('chatter', String)
    r = n.Rate(10) # 10hz
    while not n.is_shutdown():
        str = "hello world %s"%n.get_time()
        n.loginfo(str)
        pub.publish(str)
        r.sleep()

Authors: Ken Conley and Blaise Gassend
"""

import roslib; roslib.load_manifest('ronin')

import rospy

import roslib.masterapi

import rospy.masterslave
import rospy.registration

# hot patch: convince node api that it's already registered
rospy.masterslave.ROSHandler._is_registered = lambda x: True

_MASTER_CHECK_INTERVAL = 10 # seconds

def init_node(name, **kwds):
    kwds['disable_rosout'] = True
    kwds['disable_rostime'] = True
    
    rospy.init_node(name, **kwds)
    handle = RoninHandle(name, rospy.get_node_uri(), rospy.Duration(_MASTER_CHECK_INTERVAL))
    
    # hot patch
    rospy.registration.RegManager.reg_added = handle._reg_added

    handle._check_master()
    return handle


def _checked_publish(self, *args, **kwds):
    self.ronin._check_master()
    self.ronin_publish(*args[1:], **kwds)

class _PubHandle(object):
    def __init__(self, pub, handle):
        self.pub = pub
        self.handle = handle
        
    def __call__(self, *args, **kwds):
        self.handle._check_master()
        self.pub.ronin_publish(*args, **kwds)
    
class RoninHandle(object):

    def __init__(self, name, uri, check_interval):
        self._uri = uri
        self._name = name
        self._check_interval = check_interval
        # XMLRPC is stateless, so we can go-ahead and init
        self._master = roslib.masterapi.Master(name)

        self._pub_list = []
        self._sub_list = []
        
        self._next_check_time = None
        self._master_pid = None

        self.Rate = rospy.Rate
        self.Subscriber = rospy.Subscriber

        self.is_shutdown = rospy.is_shutdown
        self.get_time = rospy.get_time
        self.get_rostime = rospy.get_rostime
        self.sleep = rospy.sleep
        
        self.loginfo = rospy.loginfo
        self.logerr = rospy.logerr        
        self.logdebug = rospy.logdebug
        self.logwarn = rospy.logwarn        
        
    def _reg_added(self, resolved_name, data_type_or_uri, reg_type):
        try:
            if reg_type == rospy.registration.Registration.PUB:
                self._pub_list.append((resolved_name, data_type_or_uri))
                if self._master_pid is not None:
                    master.registerPublisher(resolved_name, data_type_or_uri, our_uri)
            elif reg_type == rospy.registration.Registration.SUB:
                self._sub_list.append((resolved_name, data_type_or_uri))
                if self._master_pid is not None:
                    master.registerSubscriber(resolved_name, data_type_or_uri, our_uri)
        except:
            pass
        return True

    def _checked_publish(self, *args, **nargs):
        self.ronin._check_master()
        original_Publisher_publish(*args, **nargs)

    def Publisher(self, *args, **kwds):
        # wrap the publisher with a call to check_master before actually publishing
        pub = rospy.Publisher(*args, **kwds)
        pub.ronin_publish = pub.publish
        pub.publish = _PubHandle(pub, self)
        return pub

    def _check_master(self):
        # NOTE: it's not possible for a ronin node to be on simtime,
        # so this is just wallclock convenience.
        now = rospy.Time.now()
        next_check_time = self._next_check_time
        if next_check_time == None or now > next_check_time:
            self._next_check_time = now + self._check_interval
            if self._master_pid != None:
                try:
                    new_pid = master.getPid()
                    if new_pid != self._master_pid:
                        self._master_pid = None
                except:
                    self._master_pid = None
            if self._master_pid == None:
                try:
                    for name, type in self._pub_list:
                        self._master.registerPublisher(name, type, self._uri)
                    for name, type in self._sub_list:
                        self._master.registerSubscriber(name, type, self._uri)

                    self._master_pid = self._master.getPid()
                except Exception, e:
                    pass
        if self._master_pid:
            return True


