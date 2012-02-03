#!/usr/bin/env python
import roslib; roslib.load_manifest('app_manager_android')
import roslib.network
import roslib.rosenv
import pyqrnative

import urlparse
urlp = urlparse.urlparse(roslib.rosenv.get_master_uri())
address = roslib.network.get_local_address()

text = "http://%s:%s"%(address, urlp.port)
print text
pyqrnative.show_data(text)
