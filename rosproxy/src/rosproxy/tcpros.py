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
# Revision $Id: __init__.py 10467 2010-07-21 22:20:07Z kwc $

"""Replacement for default rospy TCPROSHandler"""

#TODO: this should be a near drop-in replacement for the existing
#handler as long as the topic_manager is passed into the
#constructor. This modified implementation assumes that the TCPROS
#server is already started. In the past, we lazy initialized the
#server, but this optimization cannot be used in practice so we should
#simplify.

import rospy
from rospy.core import rospyerr
import rospy.exceptions
import rospy.names
import rospy.impl.transport
from rospy.impl.tcpros_pubsub import TCPROSPub, TCPROSSub, TCPROSTransport
from rospy.impl.tcpros_base import TCPROS

class ProxyTCPROSHandler(rospy.impl.transport.ProtocolHandler):
    """
    ROS Protocol handler for TCPROS. Accepts both TCPROS topic
    connections as well as ROS service connections over TCP. TCP server
    socket is run once start_server() is called -- this is implicitly
    called during init_publisher().
    """

    def __init__(self, topic_manager, tcpros_server):
        """ctor"""
        self.tcp_nodelay_map = {} # { topic : tcp_nodelay}
        self.topic_manager = topic_manager
        self.tcpros_server = tcpros_server
    
    def set_tcp_nodelay(self, resolved_name, tcp_nodelay):
        """
        @param resolved_name: resolved topic name
        @type  resolved_name: str

        @param tcp_nodelay: If True, sets TCP_NODELAY on publisher's
        socket (disables Nagle algorithm). This results in lower
        latency publishing at the cost of efficiency.
        @type  tcp_nodelay: bool
        """
        self.tcp_nodelay_map[resolved_name] = tcp_nodelay

    def shutdown(self):
        """
        stops the TCP/IP server responsible for receiving inbound connections        
        """
        pass

    def create_transport(self, resolved_name, pub_uri, protocol_params):
        """
        Connect to topic resolved_name on Publisher pub_uri using TCPROS.
        @param resolved_name str: resolved topic name
        @type  resolved_name: str
        @param pub_uri: XML-RPC URI of publisher 
        @type  pub_uri: str
        @param protocol_params: protocol parameters to use for connecting
        @type protocol_params: [XmlRpcLegal]
        @return: code, message, debug
        @rtype: (int, str, int)
        """
        
        #Validate protocol params = [TCPROS, address, port]
        if type(protocol_params) != list or len(protocol_params) != 3:
            return 0, "ERROR: invalid TCPROS parameters", 0
        if protocol_params[0] != TCPROS:
            return 0, "INTERNAL ERROR: protocol id is not TCPROS: %s"%id, 0
        id, dest_addr, dest_port = protocol_params

        sub = self.topic_manager.get_subscriber_impl(resolved_name)

        #Create connection 
        try:
            protocol = TCPROSSub(resolved_name, sub.data_class, \
                                 queue_size=sub.queue_size, buff_size=sub.buff_size,
                                 tcp_nodelay=sub.tcp_nodelay)
            conn = TCPROSTransport(protocol, resolved_name)
            # timeout is really generous. for now just choosing one that is large but not infinite
            conn.connect(dest_addr, dest_port, pub_uri, timeout=60.)
            t = threading.Thread(name=resolved_name, target=conn.receive_loop, args=(sub.receive_callback,))
            # don't enable this just yet, need to work on this logic
            #rospy.core._add_shutdown_thread(t)
            t.start()
        except rospy.exceptions.TransportInitError, e:
            rospyerr("unable to create subscriber transport: %s", e)
            return 0, "Internal error creating inbound TCP connection for [%s]: %s"%(resolved_name, e), -1

        # Attach connection to _SubscriberImpl
        if sub.add_connection(conn): #pass tcp connection to handler
            return 1, "Connected topic[%s]. Transport impl[%s]"%(resolved_name, conn.__class__.__name__), dest_port
        else:
            conn.close()
            return 0, "ERROR: Race condition failure: duplicate topic subscriber [%s] was created"%(resolved_name), 0

    def supports(self, protocol):
        """
        @param protocol: name of protocol
        @type protocol: str
        @return: True if protocol is supported
        @rtype: bool
        """
        return protocol == TCPROS
    
    def get_supported(self):
        """
        Get supported protocols
        """
        return [[TCPROS]]
        
    def init_publisher(self, resolved_name, protocol):
        """
        Initialize this node to receive an inbound TCP connection,
        i.e. startup a TCP server if one is not already running.
        
        @param resolved_name: topic name
        @type  resolved__name: str
        
        @param protocol: negotiated protocol
          parameters. protocol[0] must be the string 'TCPROS'
        @type  protocol: [str, value*]
        @return: (code, msg, [TCPROS, addr, port])
        @rtype: (int, str, list)
        """
        if protocol[0] != TCPROS:
            return 0, "Internal error: protocol does not match TCPROS: %s"%protocol, []

        addr, port = self.tcpros_server.get_address()
        return 1, "ready on %s:%s"%(addr, port), [TCPROS, addr, port]

    def topic_connection_handler(self, sock, client_addr, header):
        """
        Process incoming topic connection. Reads in topic name from
        handshake and creates the appropriate L{TCPROSPub} handler for the
        connection.
        @param sock: socket connection
        @type sock: socket.socket
        @param client_addr: client address
        @type client_addr: (str, int)
        @param header: key/value pairs from handshake header
        @type header: dict
        @return: error string or None 
        @rtype: str
        """
        for required in ['topic', 'md5sum', 'callerid']:
            if not required in header:
                return "Missing required '%s' field"%required
        else:
            resolved_topic_name = header['topic']
            md5sum = header['md5sum']
            tm = self.topic_manager
            topic = tm.get_publisher_impl(resolved_topic_name)
            if not topic:
                return "[%s] is not a publisher of  [%s]. Topics are %s"%(rospy.names.get_caller_id(), resolved_topic_name, tm.get_publications())
            elif md5sum != rospy.names.TOPIC_ANYTYPE and md5sum != topic.data_class._md5sum:

                actual_type = topic.data_class._type

                # check to see if subscriber sent 'type' header. If they did, check that
                # types are same first as this provides a better debugging message
                if 'type' in header:
                    requested_type = header['type']
                    if requested_type != actual_type:
                        return "topic types do not match: [%s] vs. [%s]"%(requested_type, actual_type)
                else:
                    # defaults to actual type
                    requested_type = actual_type

                return "Client [%s] wants topic [%s] to have datatype/md5sum [%s/%s], but our version has [%s/%s] Dropping connection."%(header['callerid'], resolved_topic_name, requested_type, md5sum, actual_type, topic.data_class._md5sum)

            else:
                #TODO:POLLING if polling header is present, have to spin up receive loop as well

                # #1334: tcp_nodelay support from subscriber option
                if 'tcp_nodelay' in header:
                    tcp_nodelay = True if header['tcp_nodelay'].strip() == '1' else False
                else:
                    tcp_nodelay = self.tcp_nodelay_map.get(resolved_topic_name, False)

                rospy.impl.tcpros_pubsub._configure_pub_socket(sock, tcp_nodelay)
                protocol = TCPROSPub(resolved_topic_name, topic.data_class, is_latch=topic.is_latch, headers=topic.headers)
                transport = TCPROSTransport(protocol, resolved_topic_name)
                transport.set_socket(sock, header['callerid'])
                transport.write_header()
                topic.add_connection(transport)
            

