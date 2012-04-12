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
# Revision $Id: __init__.py 11414 2010-10-05 23:39:47Z tfield $

"""Node that proxies ROS topics and services in order to bridge firewalls"""

import rospy

import rostopic
import rosservice

import rosproxy.handler
import rosproxy.tcpros

# for service_connection_handler
import roslib.message
import rospy.names
import thread
from rospy.impl.registration import Registration
from rospy.impl.tcpros_base import TCPROSTransport, TCPROSTransportProtocol, TCPROSServer
from rospy.service import ServiceManager
from rospy.impl.tcpros_service import ServiceImpl
from rosgraph.xmlrpc import XmlRpcNode
import rosgraph.network
import rosgraph

from new import classobj

import rospy.impl.registration 

def pub_forwarder(msg, pub):
    """
    Forward a message to a supplied publisher
    """
    pub.publish(msg)

# variant of AnyMsg that initializes from a msg_class
class PassthroughServiceMessage(roslib.message.Message):
    
    def __init__(self):
        self._buff = None

    def serialize(self, buff):
        if self._buff is None:
            raise rospy.exceptions("AnyMsg is not initialized")
        else:
            buff.write(self._buff)

    def deserialize(self, str):
        self._buff = str
        return self
        
class Proxy(object):
    
    def __init__(self):
        
        self.services = {}
        
        self.service_manager = ServiceManager(registration_listeners=rospy.impl.registration.RegistrationListeners())
        self.topic_manager = rospy.topics._TopicManager()

        self.subs_internal = {}
        
        self.external_node = None
        self.external_tcpros = None

    def connect(self):
        self.configure_internal()
        self.configure_external()
        
    def _configure_proxy_services(self, external_tcpros):
        external_tcpros.service_connection_handler = self.service_connection_handler

        # override the registration listeners to do nothing so we don't appear to be actually registering this listeners
        # create proxy handlers for each service
        i = 0
        for name, proxy in self.services.iteritems():
            
            i += 1
            c = proxy.service_class
            service = ServiceImpl(name, proxy.service_class, proxy)
            
            rospy.loginfo("registering proxied service %s"%(name))
            self.service_manager.register(name, service)
        
    def _configure_proxy_topics(self, external_tcpros, tcpros_handler):
        external_tcpros.topic_connection_handler = tcpros_handler.topic_connection_handler

        import rospy.topics
        
        # create publishers for each of the topics we internally subscribe to
        subs_initialized = {}
        for resolved_name, sub in self.subs_internal.iteritems():
            # create a pub handle
            rospy.loginfo("create external publisher [%s]", resolved_name)
            pub = self.topic_manager.acquire_impl(Registration.PUB, resolved_name, sub.data_class)
            # clone the subscriber handle, this time with a callback
            sub_ex = rospy.Subscriber(resolved_name, sub.data_class, pub_forwarder, pub)
            subs_initialized[resolved_name] = sub_ex
            # decrement the ref count of the old handle
            #sub.unregister()
            
        # throw away old subs_internal dictionary and replace it with
        # the initialized objects
        del self.subs_internal
        self.subs_internal = subs_initialized

        
    def configure_external(self):

        
        # Start another TCPROSServer to handle requests on the external port
        #  - TODO: allow handlers these to be passed into TCPROSServer constructor
        self.external_tcpros = external_tcpros = TCPROSServer()
        tcpros_handler = rosproxy.tcpros.ProxyTCPROSHandler(self.topic_manager, external_tcpros)

        self._configure_proxy_services(external_tcpros)
        self._configure_proxy_topics(external_tcpros, tcpros_handler)

        tcpros_port = rospy.get_param('~tcpros_port', 11312)
        xmlrpc_port = rospy.get_param('~xmlrpc_port', 11313)

        rospy.loginfo("reading port configuration: TCPROS[%s] XMLRPC[%s]"%(tcpros_port, xmlrpc_port))
        
        external_tcpros.start_server(port=tcpros_port)

        # TODO: this may report the address of the wrong interface
        rospy.loginfo("ROSRPC URI is rosrpc://%s:%s"%(rosgraph.network.get_local_address(), tcpros_port))

        # Startup XMLRPC interface so we can do pub/sub
        master_uri = rosgraph.get_master_uri()
        name = 'proxy-proxy'

        
        protocol_handlers = [tcpros_handler]
        rpc_handler = rosproxy.handler.ProxyHandler(name, master_uri, self.topic_manager, protocol_handlers)
        self.external_node = external_node= XmlRpcNode(xmlrpc_port, rpc_handler)

        # - start the node and wait for init
        external_node.start()
        import time
        timeout_t = time.time() + 10.
        while time.time() < timeout_t and external_node.uri is None:
            time.sleep(0.01)

        rospy.loginfo("XMLRPC interface is up %s"%self.external_node.uri)
        

    def _configure_internal_services(self, service_names):
        """
        Create rospy handles to all of the services that we are
        configured to be able to call.
        """
        
        i = 0
        for name in service_names:
            rospy.loginfo("configuring service %s", name)
            resolved_name = rospy.resolve_name(name)
            rospy.wait_for_service(name, timeout=60.)
            type_ = rosservice.get_service_type(resolved_name)
            if type_ is None:
                raise rospy.ROSInitException("cannot proxy service [%s]: unknown type"%resolved_name)

            i += 1

            # for efficiency, we generate a passthrough service
            # definition that does not do any serializatoin on the
            # request or response. This requires more work because the
            # instantiated class has to have the correct properties.
            real_service_class = roslib.message.get_service_class(type_)
            real_req = real_service_class._request_class
            real_resp = real_service_class._response_class
            request_d = {
                '_type': real_req._type,
                '_md5sum': real_req._md5sum,   
                '_full_text': real_req._full_text,   
                }
            response_d = {
                '_type': real_resp._type,
                '_md5sum': real_resp._md5sum,   
                '_full_text': real_resp._full_text,   
                }
            service_class = classobj('s_passthrough_%s'%i, (object,), {
                    '_type' : real_service_class._type,
                    '_md5sum' : real_service_class._md5sum,
                    '_request_class' : classobj('passthrough_request_%s'%i, (PassthroughServiceMessage, ), request_d),
                    '_response_class' : classobj('passthrough_response_%s'%i, (PassthroughServiceMessage,), response_d),
                    })
            
            self.services[resolved_name] = rospy.ServiceProxy(name, service_class, persistent=True)

            rospy.loginfo("proxying %s", resolved_name)
        
    def _configure_internal_topics(self, pub_names):
        """
        Create subscriptions to all the topics that we externally publish.
        """
        
        i = 0
        for name in pub_names:
            resolved_name = rospy.resolve_name(name)
            rospy.loginfo("configuring internal subscriber [%s]", resolved_name)            

            try:
                real_msg_class, _, _ = rostopic.get_topic_class(resolved_name)
            except rostopic.ROSTopicException:
                raise rospy.ROSInitException("cannot proxy subscription [%s]: unknown topic type"%resolved_name)

            i += 1
            topic_class = classobj('t_passthrough_%s'%i, (rospy.msg.AnyMsg,), {
                    '_type' : real_msg_class._type,
                    '_md5sum' : real_msg_class._md5sum,
                    })
            self.subs_internal[resolved_name] = rospy.Subscriber(name, topic_class)

            rospy.loginfo("proxying %s", resolved_name)
        
        
    def configure_internal(self):
        """
        Bring up connections to internal ROS graph
        """
        
        rospy.init_node('proxy')

        # fetch all the parameters for our node
    
        service_names = rospy.get_param('~services', {})
        self._configure_internal_services(service_names)

        pub_names = rospy.get_param('~pubs', {})
        self._configure_internal_topics(pub_names)
        

    def service_connection_handler(self, sock, client_addr, header):
        """
        @param sock: socket connection
        @type  sock: socket
        @param client_addr: client address
        @type  client_addr: (str, int)
        @param header: key/value pairs from handshake header
        @type  header: dict
        @return: error string or None 
        @rtype: str
        """

        # This is a cturtle hack. rospy's service_connection_handler
        # is wired to the ServiceManager singleton. If we replace the
        # singleton with something more configurable, then we simply
        # have to run our own ServiceManager to handle the forwarding
        # behavior.

        for required in ['service', 'md5sum', 'callerid']:
            if not required in header:
                return "Missing required '%s' field"%required
        else:
            #logger.debug("connection from %s:%s", client_addr[0], client_addr[1])
            service_name = header['service']

            sm = self.service_manager
            md5sum = header['md5sum']
            service = sm.get_service(service_name)
            if not service:
                return "[%s] is not a provider of  [%s]"%(rospy.names.get_caller_id(), service_name)
            elif md5sum != rospy.names.SERVICE_ANYTYPE and md5sum != service.service_class._md5sum:
                return "request from [%s]: md5sums do not match: [%s] vs. [%s]"%(header['callerid'], md5sum, service.service_class._md5sum)
            else:
                transport = TCPROSTransport(service.protocol, service_name, header=header)
                transport.set_socket(sock, header['callerid'])
                transport.write_header()
                thread.start_new_thread(service.handle, (transport, header))
        
def rosproxy_main():
    
    p = Proxy()
    p.connect()

    # wait for shutdown
    rospy.spin()
