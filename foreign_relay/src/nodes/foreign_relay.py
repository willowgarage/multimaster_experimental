#!/usr/bin/env python

import roslib
roslib.load_manifest('foreign_relay')

import sys
import os
import time
import xmlrpclib
import urlparse

USAGE = 'foreign_relay.py {adv|sub} topic foreign_master_uri'

g_publishers = {}

# Clean up the stuff that we registered
def shutdown(foreign_master_uri):
    server = xmlrpclib.ServerProxy(foreign_master_uri)
    for p in g_publishers:
        server.unregisterPublisher(p, topic, g_publishers[p])
        print 'unregistering %s @ %s'%(p,g_publishers[p])

def go(mode, topic, foreign_master_uri, update_period):
    publishers = {}
    if mode == 'sub':
        sub_uri = foreign_master_uri
        adv_uri = os.environ['ROS_MASTER_URI']
    else:
        sub_uri = os.environ['ROS_MASTER_URI']
        adv_uri = foreign_master_uri
    server_sub = xmlrpclib.ServerProxy(sub_uri)
    server_adv = xmlrpclib.ServerProxy(adv_uri)
    while 1:
        code, msg, topics = server_sub.getPublishedTopics('', '')
        # Determine the type of the topic
        type = None
        for t in topics:
            if t[0] == topic:
                type = t[1]
                break
        # Find publishers
        if type is not None:
            new_publishers = {}
            code, msg, state = server_sub.getSystemState('')
            pubs = state[0]
            for t in pubs:
                if t[0] == topic:
                    for p in t[1]:
                        # Hack to avoid circular re-subscription
                        dup = False
                        for pp in publishers:
                            if p.startswith(pp):
                                #print '%s is dup of %s; skipping'%(p,pp)
                                dup = True
                                break
                        if dup:
                            continue
                        code, msg, uri = server_sub.lookupNode('', p)
                        # Uniquify by master
                        sp = urlparse.urlsplit(sub_uri)
                        mangled_name = '%s_%s_%s'%(p,sp.hostname,sp.port)
                        new_publishers[mangled_name] = uri
            # Update remote advertisements appropriately
            subtractions = set(publishers.keys()) - set(new_publishers.keys())
            additions = set(new_publishers.keys()) - set(publishers.keys())
            for s in subtractions:
                server_adv.unregisterPublisher(s, topic, publishers[s])
                print 'unregistering %s @ %s'%(s,publishers[s])
            for a in additions:
                server_adv.registerPublisher(a, topic, type, new_publishers[a])
                print 'registering %s @ %s'%(a,new_publishers[a])
            publishers = new_publishers
            global g_publishers
            g_publishers = publishers
        time.sleep(update_period)

def parse(argvin):
    argv = []
    # Remove the ROS arguments.
    for arg in argvin:
        if arg.find(":=") == -1:
            argv.append(arg)
    if len(argv) != 4:
        print USAGE
        return None
    mode = argv[1]
    if mode != 'sub' and mode != 'adv':
        print USAGE
        return None
    topic = argv[2]
    foreign_master_uri = argv[3]
    return (mode, topic, foreign_master_uri)

if __name__ == '__main__':
    ret = parse(sys.argv)
    if ret is None:
        sys.exit(1)
    mode, topic, foreign_master_uri = ret
    update_period = 1.0
    try:
        go(mode, topic, foreign_master_uri, update_period)
    except KeyboardInterrupt, e:
        if mode == 'adv':
            shutdown(foreign_master_uri)
        else:
            shutdown(os.environ['ROS_MASTER_URI'])
