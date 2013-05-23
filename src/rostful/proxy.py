import roslib
roslib.load_manifest('rostful')
import rospy

import urllib2, json

from importlib import import_module

import message_conversion as msgconv

class IndividualServiceProxy:
	def __init__(self,url,name,service_type):
		self.url = url
		self.name = name
		
		service_type_module,service_type_name = tuple(service_type.split('/'))
		srv_module = import_module(service_type_module + '.srv')
		
		self.rostype = getattr(srv_module,service_type_name)
		self.rostype_req = getattr(srv_module,service_type_name + 'Request')
		self.rostype_resp = getattr(srv_module,service_type_name + 'Response')
		
		self.proxy = rospy.Service(self.name,self.rostype,self.call)
	
	def call(self,rosreq):
		req = msgconv.extract_values(rosreq)
		reqs = json.dumps(req)
		
		wsreq = urllib2.Request(self.url,data=reqs,headers = {'Content-type': 'application/json'})
		try:
			wsres = urllib2.urlopen(wsreq)
		except Exception, e:
			raise e
		
		if wsres.getcode() != 200:
			#TODO: flip out
			pass
		
		data_str = wsres.read().strip()
		#print data_str
		data = json.loads(data_str)
		
		rosresp = self.rostype_resp()
		msgconv.populate_instance(data,rosresp)
		
		return rosresp
		

class RostfulServiceProxy:
	def __init__(self,url,remap=False):
		if url.endswith('/'):
			url = url[:-1]
		self.url = url
		
		config_url = self.url + '/_config'
		
		req = urllib2.Request(config_url)
		
		try:
			res = urllib2.urlopen(req)
		except Exception, e:
			#TODO: flip out
			pass
		
		if res.getcode() != 200:
			#TODO: flip out
			pass
		
		data = json.loads(res.read().strip())
		
		if data['type'] == 'manifest':
			for service in data['services']:
				self.setup_service(self.url + '/' + service['url'],service,remap)
		elif data['type'] == 'service':
			self.setup_service(self.url,data,remap)
		else:
			#TODO: flip out
			pass
	
	def setup_service(self,service_url,service,remap=False):
		name = service['url']
		if remap:
			name = name + '_ws'
		service_type = service['rostype']
		
		IndividualServiceProxy(service_url,name,service_type)
