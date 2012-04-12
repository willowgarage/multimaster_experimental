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
# Revision $Id: handler.py 11413 2010-10-05 23:28:20Z kwc $

import os
import sys
import socket
import threading
import traceback
import time
import urlparse

from rosgraph.xmlrpc import XmlRpcHandler

import rospy.names

import rospy.impl.tcpros

from rospy.core import global_name, is_topic
from rospy.impl.registration import get_topic_manager
from rospy.impl.validators import non_empty, ParameterInvalid

from rospy.impl.masterslave import apivalidate

# pseudo-validators ###############################
# these validators actually return tuples instead of a function and it is up to a custom
# validator on the class itself to perform the validation
def is_publishers_list(paramName):
    return ('is_publishers_list', paramName)

class ProxyHandler(XmlRpcHandler):

    def __init__(self, name, master_uri, topic_manager, protocol_handlers):
        """
        Variant handler for proxy
        @param name: ROS name of this node
        @type  name: str
        @param master_uri: URI of master node, or None if this node is the master
        @type  master_uri: str
        """
        super(ProxyHandler, self).__init__()
        self.master_uri = master_uri
        self.name = name
        self.uri = None
        self.done = False

        self.protocol_handlers = protocol_handlers
            
        self.reg_man = None
        
        # this is the key difference from the normal handler
        self.topic_man = topic_manager

    ###############################################################################
    # INTERNAL 

    def _ready(self, uri):
        """
        @param uri: XML-RPC URI
        @type  uri: str
        callback from ROSNode to inform handler of correct i/o information
        """
        self.uri = uri

    def _custom_validate(self, validation, param_name, param_value, caller_id):
        """
        Implements validation rules that require access to internal ROSHandler state.
        @param validation: name of validation rule to use
        @type  validation: str
        @param param_name: name of parameter being validated
        @type  param_name: str
        @param param_value str: value of parameter
        @type  param_value: str
        @param caller_id: value of caller_id parameter to API method
        @type  caller_id: str
        @raise ParameterInvalid: if the parameter does not meet validation
        @return: new value for parameter, after validation
        """
        if validation == 'is_publishers_list':
            if not type(param_value) == list:
                raise ParameterInvalid("ERROR: param [%s] must be a list"%param_name)
            for v in param_value:
                if not isinstance(v, basestring):
                    raise ParameterInvalid("ERROR: param [%s] must be a list of strings"%param_name)
                parsed = urlparse.urlparse(v)
                if not parsed[0] or not parsed[1]: #protocol and host
                    raise ParameterInvalid("ERROR: param [%s] does not contain valid URLs [%s]"%(param_name, v))
            return param_value
        else:
            raise ParameterInvalid("ERROR: param [%s] has an unknown validation type [%s]"%(param_name, validation))

    ###############################################################################
    # EXTERNAL API

    @apivalidate([])
    def getBusStats(self, caller_id):
        # not supported
        return 1, '', [[], [], []]

    @apivalidate([])
    def getBusInfo(self, caller_id):
        # not supported
        return 1, '', [[], [], []]
    
    @apivalidate('')
    def getMasterUri(self, caller_id):
        return 1, self.master_uri, self.master_uri
        
    @apivalidate(0, (None, ))
    def shutdown(self, caller_id, msg=''):
        return -1, "not authorized", 0

    @apivalidate(-1)
    def getPid(self, caller_id):
        return -1, "not authorized", 0

    ###############################################################################
    # PUB/SUB APIS

    @apivalidate([])
    def getSubscriptions(self, caller_id):
        """Retrieve a list of topics that this node subscribes to
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @return [int, str, [ [topic1, topicType1]...[topicN, topicTypeN]]]: list of topics this node subscribes to
        """
        return 1, "subscriptions", self.topic_man.get_subscriptions()

    @apivalidate([])
    def getPublications(self, caller_id):
        """
        Retrieve a list of topics that this node publishes.
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @return: list of topics published by this node
        @rtype: [int, str, [ [topic1, topicType1]...[topicN, topicTypeN]]]
        """
        return 1, "publications", self.topic_man.get_publications()
    
    @apivalidate(-1, (global_name('parameter_key'), None))
    def paramUpdate(self, caller_id, parameter_key, parameter_value):
        """
        Callback from master of current publisher list for specified topic.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param parameter_key str: parameter name, globally resolved
        @type  parameter_key: str
        @param parameter_value New parameter value
        @type  parameter_value: XMLRPC-legal value
        @return: [code, status, ignore]. If code is -1 ERROR, the node
        is not subscribed to parameter_key
        @rtype: [int, str, int]
        """
        # not supported
        return -1, 'not authorized', 0

    @apivalidate(-1, (is_topic('topic'), is_publishers_list('publishers')))
    def publisherUpdate(self, caller_id, topic, publishers):
        """
        Callback from master of current publisher list for specified topic.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param topic str: topic name
        @type  topic: str
        @param publishers: list of current publishers for topic in the form of XMLRPC URIs
        @type  publishers: [str]
        @return: [code, status, ignore]
        @rtype: [int, str, int]
        """
        if self.reg_man:
            for uri in publishers:
                self.reg_man.publisher_update(topic, publishers)
    
    @apivalidate([], (is_topic('topic'), non_empty('protocols')))
    def requestTopic(self, caller_id, topic, protocols):
        """
        Publisher node API method called by a subscriber node.
   
        Request that source allocate a channel for communication. Subscriber provides
        a list of desired protocols for communication. Publisher returns the
        selected protocol along with any additional params required for
        establishing connection. For example, for a TCP/IP-based connection,
        the source node may return a port number of TCP/IP server. 
        @param caller_id str: ROS caller id    
        @type  caller_id: str
        @param topic: topic name
        @type  topic: str
        @param protocols: list of desired
         protocols for communication in order of preference. Each
         protocol is a list of the form [ProtocolName,
         ProtocolParam1, ProtocolParam2...N]
        @type  protocols: [[str, XmlRpcLegalValue*]]
        @return: [code, msg, protocolParams]. protocolParams may be an
        empty list if there are no compatible protocols.
        @rtype: [int, str, [str, XmlRpcLegalValue*]]
        """
        if not self.topic_man.has_publication(topic):
            return -1, "Not a publisher of [%s]"%topic, []
        for protocol in protocols: #simple for now: select first implementation 
            protocol_id = protocol[0]
            for h in self.protocol_handlers:
                if h.supports(protocol_id):
                    return h.init_publisher(topic, protocol)
        return 0, "no supported protocol implementations", []

    
