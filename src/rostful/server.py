import roslib
roslib.load_manifest('rostful')
import rospy
from rospy.service import ServiceManager
import rosservice

from importlib import import_module

import json
import sys

JSON = True
if JSON:
	import message_conversion as msgconv
else:
	from StringIO import StringIO

from wsgiref.simple_server import make_server

class Service:
	def __init__(self,service_name,service_type):
		self.name = service_name
		
		service_type_module,service_type_name = tuple(service_type.split('/'))
		srv_module = import_module(service_type_module + '.srv')
		
		self.rostype_name = service_type
		self.rostype = getattr(srv_module,service_type_name)
		self.rostype_req = getattr(srv_module,service_type_name + 'Request')
		self.rostype_resp = getattr(srv_module,service_type_name + 'Response')
		
		self.proxy = rospy.ServiceProxy(self.name, self.rostype)
	
	def call(self,req):
		rosreq = self.rostype_req()
		if JSON:
			msgconv.populate_instance(req,rosreq)
		else:
			rosreq.deserialize(req)
		
		fields = []
		for slot in rosreq.__slots__:
			fields.append(getattr(rosreq,slot))
		fields = tuple(fields)
		
		rosresp = self.proxy(*fields)
		
		if JSON:
			resp = msgconv.extract_values(rosresp)
		else:
			resp = StringIO()
			rosresp.serialize(resp)
			resp = resp.getvalue()
		return resp

CONFIG_PATH = '_config'

class RostfulServer:
	def __init__(self):
		self.services = {}
		
	def add_service(self,service_name,ws_name = None):
		resolved_service_name = rospy.resolve_name(service_name)
		service_type = rosservice.get_service_type(resolved_service_name)
		
		if ws_name is None:
			ws_name = service_name
		if ws_name.startswith('/'):
			ws_name = ws_name[1:]
		
		self.services[ws_name] = Service(service_name,service_type)
	
	def wsgifunc(self):
		return self._handle
	
	def _handle(self,environ,start_response):
		if environ['REQUEST_METHOD'] == 'GET':
			return self._handle_get(environ,start_response)
		elif environ['REQUEST_METHOD'] == 'POST':
			return self._handle_post(environ,start_response)
		else:
			#TODO: flip out
			pass
	
	def _handle_get(self,environ,start_response):
		path = environ['PATH_INFO']
		print path
		if not path.endswith('/' + CONFIG_PATH):
			print '404'
			status = '404 Not Found'
			headers = [('Content-type', 'text/plain')]
			start_response(status, headers)
			return []
		if path == '/' + CONFIG_PATH:
			d = {'type': 'manifest', 'services': [], 'topics': []}
			for service_name, service in self.services.iteritems():
				srv_d = {'url': service_name, 'rostype': service.rostype_name}
				d['services'].append(srv_d)
			json_config = json.dumps(d)
			
			status = '200 OK'
			headers = [('Content-type', 'application/json'),('Content-length',str(len(json_config)))]
			start_response(status, headers)
			return json_config
		else:
			name = path[1:-(len(CONFIG_PATH)+1)]
			if self.services.has_key(name):
				service_name = name
				
				service = self.services[service_name]
				d = {'type': 'service', 'url': service_name, 'rostype': service.rostype_name}
				json_config = json.dumps(d)
				
				status = '200 OK'
				headers = [('Content-type', 'application/json'),('Content-length',str(len(json_config)))]
				start_response(status, headers)
				return json_config
			else:
				status = '404 Not Found'
				headers = [('Content-type', 'text/plain')]
				start_response(status, headers)
				return []
		
	def _handle_post(self,environ,start_response):
		service_name =  environ['PATH_INFO'][1:]
		if not self.services.has_key(service_name):
			status = '404 Not Found'
			headers = [('Content-type', 'text/plain')]
			start_response(status, headers)
			return []
		service = self.services[service_name]
		
		length = int(environ['CONTENT_LENGTH'])
		req = json.loads(environ['wsgi.input'].read(length))
		#from test_ros.srv import *
		#rosreq = AddTwoIntsRequest(1,2)
		#req = msgconv.extract_values(rosreq)
		
		try:
			#print 'calling service ', service.name
			
			resp = service.call(req)
			resp = json.dumps(resp)
			
			status = '200 OK'
			headers = [('Content-type', 'application/json'),('Content-length',str(len(resp)))]
			start_response(status, headers)
			return resp
		except Exception, e:
			e_str = '%s: %s' % (str(type(e)),str(e))
			
			status = '500 Internal Server Error'
			headers = [('Content-type', 'text/plain'),('Content-length',str(len(e_str)))]
			start_response(status, headers)
			return e_str
