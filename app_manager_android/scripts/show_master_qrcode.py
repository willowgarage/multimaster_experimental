#!/usr/bin/env python
import roslib; roslib.load_manifest('app_manager_android')
import rosgraph
import rosgraph.network
import pyqrnative

import urlparse
urlp = urlparse.urlparse(rosgraph.get_master_uri())
address = rosgraph.network.get_local_address()

text = "http://%s:%s"%(address, urlp.port)
print text
pyqrnative.show_data(text)
