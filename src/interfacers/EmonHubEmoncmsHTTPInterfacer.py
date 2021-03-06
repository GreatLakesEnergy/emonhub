"""class EmonHubEmoncmsHTTPInterfacer
"""
import zlib
import time
import json
import urllib2
import httplib
import redis
import msgpack

from sys import getsizeof as size
from pydispatch import dispatcher
from emonhub_interfacer import EmonHubInterfacer

class EmonHubEmoncmsHTTPInterfacer(EmonHubInterfacer):

    def __init__(self, name):
        # Initialization
        super(EmonHubEmoncmsHTTPInterfacer, self).__init__(name)

        self._name = name
        self._data_size = 0
        self._settings = {
            'subchannels':['ch1'],
            'pubchannels':['ch2'],
            'compression':True,
            'use_binary':True,
            'apikey': "",
            'url': "http://emoncms.org",
            'redis_url':'localhost',
            'redis_port': 6379,
	    'redis_db':0,
            'senddata': 1,
            'sendstatus': 0,
            'data_send_interval':60,
            'status_send_interval':260,
            'buffer_size':1000,
        }

	self._compression_level = 9
	self._batch_size = 30 #how should the data be sent
	self._retry = 5
        self.buffer = []
        self.lastsent = time.time()
        self.lastsentstatus = time.time()
	self.r = redis.Redis(self._settings['redis_url'], 
		 port=self._settings['redis_port'],
		 db=self._settings['redis_db'] )

	# We wait here until redis has successfully started up
	redisready = False
	while not redisready:
	    try:
		self.r.client_list()
		redisready = True
	    except redis.ConnectionError:
		logger.info("waiting for redis-server to start...")
		time.sleep(1.0)

    def receiver(self, cargo):

        # Create a frame of data in "emonCMS format"
        f = []
        f.append(int(cargo.timestamp))
        f.append(cargo.nodeid)
        for i in cargo.realdata:
            f.append(i)
        if cargo.rssi:
            f.append(cargo.rssi)

        self._log.debug(str(cargo.uri) + " adding frame to buffer => "+ str(f))
	# If buffer is full don't append
        # Append to bulk post buffer
	if len(self.buffer) <= self._settings['buffer_size']:
		self.buffer.append(f)
	else:
		self._log.warning("buffer full no more data points will be added")

    def action(self):

        now = time.time()

        if (now-self.lastsent) > int(self._settings['data_send_interval']):
            self.lastsent = now
            # print json.dumps(self.buffer)
            if int(self._settings['senddata']):
                # Send bulk post
		counter = 0
		while self.buffer and counter < self._retry:
			#slice out a piece from the buffer
			to_send = self.buffer[:self._batch_size]
			if self.bulkpost(to_send):
			     # Clear buffer if successfull else keep buffer and try again
			     self.buffer = self.buffer[self._batch_size:]
			else:
			     self._log.warning("Failed contacting server retrying: %s"%str(counter))
			     self._log.warning("Buffer size is : %s"%str(len(self.buffer)))
			     counter = counter + 1

        if (now-self.lastsentstatus) > int(self._settings['status_send_interval']):
            self.lastsentstatus = now
            if int(self._settings['sendstatus']):
                self.sendstatus()

    def bulkpost(self, databuffer):
	"""
	Utitliy function that will prepare raw sensor data to be shipped
	"""

        self._log.info("Prepping bulk post: " + str( databuffer ))
    	#Removing length check fo apikey
        if not 'apikey' in self._settings.keys() or str.lower(str(self._settings['apikey'])) == 'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx':
            self._log.error("API key not found skipping: " + str( databuffer ))
            return False
        self._log.debug("data string %s "%databuffer)
	
        # If we are sending binary data
        if self._settings["use_binary"]:
            data_string = msgpack.packb(databuffer)
	else:
            data_string = json.dumps(databuffer, separators=(',', ':'))


        # time that the request was sent at
        sentat = int(time.time())

        # Construct post_url (without apikey)
        post_url = self._settings['url']+'/input/bulk'+'.json?apikey='
        post_body = data_string


        if self._settings["compression"]:
            post_body = zlib.compress(post_body, self._compression_level)

        # Add apikey to post_url
        post_url = post_url + self._settings['apikey'] + "&time="+str(sentat)

        # logged before apikey added for security
        self._log.debug("sending: " + post_url + " body:" +post_body)

        # The Develop branch of emoncms allows for the sending of the apikey in the post
        # body, this should be moved from the url to the body as soon as this is widely
        # adopted

        reply = self._send_post(post_url, post_body)
        if reply.lower().strip() == 'ok':
            self._log.debug("acknowledged receipt with '" + reply + "' from " + self._settings['url'])
            #Send data to LCD
            self.r.set('server:active',1)
            return True
        else:
            self.r.set('server:active',0)
            self._log.warning("send failure: wanted 'ok' but got '" +reply+ "'")
            self._log.warning("Keeping buffer till successfull attempt, buffer length: " + str(len(self.buffer)))
            return False

    def _send_post(self, post_url, post_body=None):
        """

        :param post_url:
        :param post_body:
        :return: the received reply if request is successful
        """
        """Send data to server.

        data (list): node and values (eg: '[node,val1,val2,...]')
        time (int): timestamp, time when sample was recorded

        return True if data sent correctly

        """

        reply = ""
        request = urllib2.Request(post_url, post_body)
        request.add_header('Content-Type','application/json')

        if self._settings["compression"]:
            request.add_header('Content-Encoding','gzip')
        if self._settings['use_binary']:
            request.add_header('Content-Type','application/x-msgpack')


        self._data_size = self._data_size + size(post_body)
        try:
            response = urllib2.urlopen(request, timeout=60)
        except urllib2.HTTPError as e:
            self._log.warning(self.name + " couldn't send to server, HTTPError: " +
                              str(e.code))
        except urllib2.URLError as e:
            self._log.warning(self.name + " couldn't send to server, URLError: " +
                              str(e.reason))
        except httplib.HTTPException:
            self._log.warning(self.name + " couldn't send to server, HTTPException")
        except Exception:
            import traceback
            self._log.warning(self.name + " couldn't send to server, Exception: " +
                              traceback.format_exc())
        else:
            reply = response.read()
        finally:
            self._log.debug("amount of data sent is %s"%self._data_size)
            return reply

    def sendstatus(self):
        if not 'apikey' in self._settings.keys() or str.lower(str(self._settings['apikey'])) == 'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx':
            return

        # MYIP url
        post_url = self._settings['url']+'/myip/set.json?apikey='
        # Print info log
        self._log.info("sending: " + post_url + "E-M-O-N-C-M-S-A-P-I-K-E-Y")
        # add apikey
        post_url = post_url + self._settings['apikey']
        # send request
        reply = self._send_post(post_url,None)

    def set(self, **kwargs):
        for key,setting in self._settings.iteritems():
            if key in kwargs.keys():
                # replace default
                self._settings[key] = kwargs[key]

        # Subscribe to internal channels
        for channel in self._settings["subchannels"]:
            dispatcher.connect(self.receiver, channel)
            self._log.debug(self._name+" Subscribed to channel' : " + str(channel))

