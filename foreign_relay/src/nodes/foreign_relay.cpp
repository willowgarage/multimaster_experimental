///////////////////////////////////////////////////////////////////////////////
// relay just passes messages on. it can be useful if you're trying to ensure
// that a message doesn't get sent twice over a wireless link, by having the 
// relay catch the message and then do the fanout on the far side of the 
// wireless link.
//
// Copyright (C) 2009, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
/////////////////////////////////////////////////////////////////////////////


#include <cstdio>
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"
#include "XmlRpc.h"
#include "ros/xmlrpc_manager.h"
#include "ros/network.h"

using std::string;
using std::vector;
using namespace topic_tools;

static ros::NodeHandle *g_node = NULL;
static bool g_advertised = false;
static string g_foreign_topic;
static string g_local_topic;
static ros::Publisher g_pub;
static string g_host;
static uint32_t g_port = 0;
static bool g_error = false;
typedef enum
{
  MODE_ADV,
  MODE_SUB
} relay_mode_t;
static relay_mode_t g_mode;

#define USAGE "USAGE: foreign_relay {adv|sub} FOREIGN_MASTER_URI FOREIGN_TOPIC LOCAL_TOPIC"

ros::XMLRPCManagerPtr g_xmlrpc_manager = ros::XMLRPCManager::instance();

void foreign_advertise(const std::string &type);
void foreign_unadvertise();
void foreign_subscribe();
void foreign_unsubscribe();
void in_cb(const boost::shared_ptr<ShapeShifter const>& msg);

void foreign_advertise(const std::string &type)
{
  XmlRpc::XmlRpcClient *client = g_xmlrpc_manager->getXMLRPCClient(g_host, g_port, "/");
  XmlRpc::XmlRpcValue args, result;
  args[0] = ros::this_node::getName();
  args[1] = g_foreign_topic;
  args[2] = type;
  args[3] = g_xmlrpc_manager->getServerURI();
  if (!client->execute("registerPublisher", args, result))
  {
    ROS_FATAL("Failed to contact foreign master at [%s:%d] to register [%s].", g_host.c_str(), g_port, g_foreign_topic.c_str());
    g_error = true;
    ros::shutdown();
  }
  ros::XMLRPCManager::instance()->releaseXMLRPCClient(client);
}

void foreign_unadvertise()
{
  XmlRpc::XmlRpcClient *client = g_xmlrpc_manager->getXMLRPCClient(g_host, g_port, "/");
  XmlRpc::XmlRpcValue args, result;
  args[0] = ros::this_node::getName();
  args[1] = g_foreign_topic;
  args[2] = g_xmlrpc_manager->getServerURI();
  if (!client->execute("unregisterPublisher", args, result))
  {
    ROS_ERROR("Failed to contact foreign master at [%s:%d] to unregister [%s].", g_host.c_str(), g_port, g_foreign_topic.c_str());
    g_error = true;
  }
  ros::XMLRPCManager::instance()->releaseXMLRPCClient(client);
}

void foreign_subscribe()
{
  XmlRpc::XmlRpcClient *client = 
          g_xmlrpc_manager->getXMLRPCClient(g_host, g_port, "/");
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  args[1] = g_foreign_topic;
  args[2] = "*";
  args[3] = g_xmlrpc_manager->getServerURI();
  if (!client->execute("registerSubscriber", args, result))
  {
    ROS_FATAL("Failed to contact foreign master at [%s:%d] to register [%s].", 
              g_host.c_str(), g_port, g_foreign_topic.c_str());
    g_error = true;
    ros::shutdown();
    return;
  }
  
  {
    // Horrible hack: the response from registerSubscriber() can contain a
    // list of current publishers.  But we don't have a way of injecting them
    // into roscpp here.  Now, if we get a publisherUpdate() from the master,
    // everything will work.  So, we ask the master if anyone is currently
    // publishing the topic, grab the advertised type, use it to advertise
    // ourselves, then unadvertise, triggering a publisherUpdate() along the
    // way.
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    args[1] = std::string("");
    if(!client->execute("getPublishedTopics", args, result))
    {
      ROS_FATAL("Failed to call getPublishedTopics() on foreign master at [%s:%d]",
                g_host.c_str(), g_port);
      g_error = true;
      ros::shutdown();
      return;
    }
    if (!ros::XMLRPCManager::instance()->validateXmlrpcResponse("getPublishedTopics", result, payload))
    {
      ROS_FATAL("Failed to get validate response to getPublishedTopics() from foreign master at [%s:%d]",
                g_host.c_str(), g_port);
      g_error = true;
      ros::shutdown();
      return;
    }
    for(int i=0;i<payload.size();i++)
    {
      std::string topic = std::string(payload[i][0]);
      std::string type = std::string(payload[i][1]);

      if(topic == g_foreign_topic)
      {
        foreign_advertise(type);
        foreign_unadvertise();
        break;
      }
    }
  }

  ros::XMLRPCManager::instance()->releaseXMLRPCClient(client);
}

void foreign_unsubscribe()
{
  XmlRpc::XmlRpcClient *client = g_xmlrpc_manager->getXMLRPCClient(g_host, g_port, "/");
  XmlRpc::XmlRpcValue args, result;
  args[0] = ros::this_node::getName();
  args[1] = g_foreign_topic;
  args[2] = g_xmlrpc_manager->getServerURI();
  if (!client->execute("unregisterSubscriber", args, result))
  {
    ROS_ERROR("Failed to contact foreign master at [%s:%d] to unregister [%s].", g_host.c_str(), g_port, g_foreign_topic.c_str());
    g_error = true;
  }
  ros::XMLRPCManager::instance()->releaseXMLRPCClient(client);
}

void in_cb(const boost::shared_ptr<ShapeShifter const>& msg)
{
  if (!g_advertised)
  {
    ROS_INFO("Received message from: %s",
             (*(msg->__connection_header))["callerid"].c_str());
    if(g_mode == MODE_SUB)
    {
      // Advertise locally
      g_pub = msg->advertise(*g_node, g_local_topic, 10);
      ROS_INFO("Advertised locally as %s, with type %s", 
               g_local_topic.c_str(),
               (*(msg->__connection_header))["type"].c_str());
    }
    else
    {
      // We advertise locally as a hack, to get things set up properly.
      g_pub = msg->advertise(*g_node, g_foreign_topic, 10);
      // Advertise at the foreign master.
      foreign_advertise((*(msg->__connection_header))["type"]);
      ROS_INFO("Advertised foreign as %s, with type %s", 
               g_foreign_topic.c_str(),
               (*(msg->__connection_header))["type"].c_str());
    }
    g_advertised = true;
  }
  g_pub.publish(msg);
}

int main(int argc, char **argv)
{
  if (argc < 5)
  {
    ROS_FATAL(USAGE);
    return 1;
  }
  if(std::string(argv[1]) == "adv")
    g_mode = MODE_ADV;
  else if(std::string(argv[1]) == "sub")
    g_mode = MODE_SUB;
  else
  {
    ROS_FATAL(USAGE);
    return 1;
  }
  std::string foreign_master_uri;
  foreign_master_uri = argv[2];
  g_foreign_topic = argv[3];
  g_local_topic = argv[4];
  std::string local_topic_basename;
  if(!getBaseName(g_local_topic, local_topic_basename))
  {
    ROS_FATAL("Failed to extract basename from topic [%s]", 
              g_local_topic.c_str());
    return 1;
  }
  if (!ros::network::splitURI(foreign_master_uri, g_host, g_port))
  {
    ROS_FATAL("Couldn't parse the foreign master URI [%s] into a host:port pair.", foreign_master_uri.c_str());
    return 1;
  }

  char buf[1024];
  snprintf(buf, sizeof(buf), "%d", ros::master::getPort());
  ros::init(argc, argv, local_topic_basename + string("_foreign_relay_") + buf + "_" + ((g_mode == MODE_ADV) ? "adv" : "sub"),
            ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  g_node = &pnh;

  ros::Subscriber sub;
  if(g_mode == MODE_SUB)
  {
    // We subscribe locally as a hack, to get our callback set up properly.
    sub = n.subscribe<ShapeShifter>(g_foreign_topic, 10, &in_cb, 
                                    ros::TransportHints().unreliable());
    // Subscribe at foreign master.
    foreign_subscribe();
  }
  else
  {
    // Subscribe at local master.
    sub = n.subscribe<ShapeShifter>(g_local_topic, 10, &in_cb);
  }

  ros::spin();

  if(g_mode == MODE_SUB)
    foreign_unsubscribe();
  else
    foreign_unadvertise();

  return g_error;
}

