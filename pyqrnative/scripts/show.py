#!/usr/bin/env python
from ros import pyqrnative
from optparse import OptionParser
NAME="show.py"
parser = OptionParser(usage="usage: %prog <data>", prog=NAME)
options, args = parser.parse_args()
text = args[0]
pyqrnative.show_data(text)
