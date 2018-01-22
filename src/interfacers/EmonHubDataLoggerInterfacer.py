import time
import serial
import Cargo
import string
from pydispatch import dispatcher
import emonhub_interfacer as ehi
from emonhub_interfacer import EmonHubInterfacerInitError

"""class EmonhubSerialInterfacer
This interfacer is for the MSRx charger controllers
Monitors the serial port, connect data logger usb cable first and switch on the bbb

"""


class EmonHubDataLoggerInterfacer(ehi.EmonHubInterfacer):
    
    #(WAIT_HEADER, IN_KEY, IN_VALUE, IN_CHECKSUM) = range(4)

    def __init__(self, name, com_port='', com_baud=9600, toextract='' , poll_interval=30):
        """Initialize interfacer

        com_port (string): path to COM port

        """

        # Initialization
        super(EmonHubDataLoggerInterfacer, self).__init__(name)

        # Open serial port
        self._ser = self._open_serial_port(com_port, com_baud)
        
        # Initialize RX buffer
        self._rx_buf = ''

	self.poll_interval = int(poll_interval)
	self.last_read = time.time()
        
        #Parser requirments
        self._extract = toextract
        #print "init system with to extract %s"%self._extract


    def close(self):
        """Close serial port"""
        
        # Close serial port
        if self._ser is not None:
            self._log.debug("Closing serial port")
            self._ser.close()

    def _open_serial_port(self, com_port, com_baud):
        """Open serial port

        com_port (string): path to COM port

        """

        #if not int(com_baud) in [75, 110, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]:
        #    self._log.debug("Invalid 'com_baud': " + str(com_baud) + " | Default of 9600 used")
        #    com_baud = 9600

        try:
            s = serial.Serial(com_port, com_baud, timeout=10)
            #request="#MD1<CR>\r"
            #s.write(request)
            self._log.debug("Opening serial port: " + str(com_port) + " @ "+ str(com_baud) + " bits/s")
        except serial.SerialException as e:
            self._log.error(e)
            raise EmonHubInterfacerInitError('Could not open COM port %s' %
                                           com_port)
        else:
            return s

    def parse_package(self,data):
        """
        Convert package from data logger string format to emonhub expected format

        """
	#we need to group data comming by index
	data = self._rx_buf
	indices =[0,4,12,17,18,19,20,21,22,23,24,25,26,27,35,36,37,38,46,48,51,55,58,61,64,67,70,73,76,79,82,86,90,96,102,108,114,115,116,117,118,122,123,127]
	
	#data = ''.join(x for x in data if x.isdigit())
	"""
	emonhub to accept our string, we need to have a numeric value of incoming regulation mode from data logger; B=Boost, E=Equalise, F=Float, N=Night, T=Test, I=Initial 
	"""
	regulation_mode = {'B':'1','E':'2','F':'3','N':'4','T':'5','I':'6'}
	string_header = "DAT"
	product_name = {'MSRx2':'00002','MSRx4':'00004','MSRx6':'00006','MSRx8':'00008'}
	load_volt = "---" #If Standard MSRx then "---"

	data = string.replace(data,string_header,'999')

	
	for i in regulation_mode.keys():
		if i in data:
			 data = string.replace(data,i,regulation_mode[i])
	
	for i in product_name.keys():
                if i in data:
                         data = string.replace(data,i,product_name[i])


	
	#for i, x in enumerate(product_name):
        #        if product_name[i] in data:
        #                 data = string.replace(data,product_name[i],product_numeric_value[i])
	
	if load_volt in data:
		data = string.replace(data,load_volt,'000')
	
	data =[data[i:j] for i,j in zip(indices,indices[1:]+[None])]
	data = ' '.join(data)
	
	data = 'ok' + ' '+ self._settings['nodeoffset'] + ' ' + data #emonhub setup reject "Ok" followed by nodeoffset
	

        clean_data = "%s"%self._settings['nodeoffset']
        if not data:
          self._log.warning("No data returned")
          return "" 
        
	data = data[:-7] #remove CR, last data of the incoming string
	
	clean_data = clean_data + " " + str(data)
	self._log.debug("++++++++++++++ %s" %(clean_data))
	return clean_data
            

    def _read_serial(self):
        self._log.debug(" Starting Serial read")
        s = serial.Serial('/dev/ttyUSB0', 9600)
        request="#MD1<CR>\r"
        s.write(request)

	try:
            self._rx_buf = self._rx_buf + self._ser.readline()
        except Exception,e:
            self._log.error(e)
            self._rx_buf = ""


        # If line incomplete, exit
        if 'END#' not in self._rx_buf:
            return

        # Remove CR,LF
        self._log.debug("RAW DATA %s" %str(self._rx_buf))

	

    def read(self):
       
	#Read data from serial port and process if complete line received.

        #Return data as a list: [NodeID, val1, val2]
        

        # Read serial RX
	now = time.time()
	if not (now - self.last_read) > self.poll_interval:
            #self._log.debug(" Waiting for %s seconds "%(str(now - self.last_read)))
            # Wait to read based on poll_interval
	    return 
	
	# Read from serial         
	self._read_serial()
	# Update last read time
        self.last_read = now
        # If line incomplete, exit
        if self._rx_buf == None:
            return

	# sample data looks like  ['DAT1', '80131345', 'MSRx2', '8', '0', 'N', '0', '0', '0', '0', '0', '0', '0', '00000000', '0', '0', '0', '00000000', '11', '492', '+215', '492', '491', '492', '493', '493', '492', '492', '492', '---', '0000', '0000', '000000', '000000', '000035', '000036', '0', '0', '0', '0', '0000', '0', 'END#', '\r\xff']

        # Create a Payload object
        c = Cargo.new_cargo(rawdata = self._rx_buf)
        f = self.parse_package(self._rx_buf)
	f = f.split()

        # Reset buffer
        self._rx_buf = ''

	if f:
	        if int(self._settings['nodeoffset']):
                    c.nodeid = int(self._settings['nodeoffset'])
                    c.realdata = f[3:] # return from position 3
                else:
                    self._log.error("nodeoffset needed in emonhub configuratio, make sure it exits ans is integer ")
                    pass
                       
        return c

