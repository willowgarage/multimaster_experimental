import roslib; roslib.load_manifest('pyqrnative')
from pyqrnative import QRCode

qr = QRCode(20, QRErrorCorrectLevel.L)
qr.addData("http://www.baconsalt.com")
qr.make()

im = qr.makeImage()

im.show()
