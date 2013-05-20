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

from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

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

class Handler(BaseHTTPRequestHandler):
	
	
	def do_GET(self):
		if not self.path.endswith('/' + CONFIG_PATH):
			self.send_error(404)
			return
		if self.path == '/' + CONFIG_PATH:
			d = {'type': 'manifest', 'services': [], 'topics': []}
			for service_name, service in self.server.services.iteritems():
				srv_d = {'url': service_name, 'rostype': service.rostype_name}
				d['services'].append(srv_d)
			json_config = json.dumps(d)
			
			self.send_response(200)
			self.send_header('Content-type','application/json')
			self.end_headers()
			
			self.wfile.write(json_config)
		else:
			name = self.path[1:-(len(CONFIG_PATH)+1)]
			if self.server.services.has_key(name):
				service_name = name
				
				service = self.server.services[service_name]
				d = {'type': 'service', 'url': service_name, 'rostype': service.rostype_name}
				json_config = json.dumps(d)
				
				self.send_response(200)
				self.send_header('Content-type','application/json')
				self.end_headers()
				
				self.wfile.write(json_config)
			else:
				self.send_error(404)
				return
	
	def do_POST(self):
		service_name =  self.path[1:]
		if not self.server.services.has_key(service_name):
			self.send_error(404)
			return
		service = self.server.services[service_name]
		length = int(self.headers.getheader('content-length'))
		req = json.loads(self.rfile.read(length))
		#from test_ros.srv import *
		#rosreq = AddTwoIntsRequest(1,2)
		#req = msgconv.extract_values(rosreq)
		
		try:
			#print 'calling service ', service.name
			
			resp = service.call(req)
			resp = json.dumps(resp)
			
			self.send_response(200)
			self.send_header('Content-type','application/json')
			self.end_headers()
			
			self.wfile.write(resp)
		except Exception, e:
			print type(e), e
			#TODO: handle error
			pass

class RostfulServer(HTTPServer):
	def __init__(self,server_address):
		HTTPServer.__init__(self, server_address, Handler)
		self.services = {}
	
	def add_service(self,service_name,ws_name = None):
		resolved_service_name = rospy.resolve_name(service_name)
		service_type = rosservice.get_service_type(resolved_service_name)
		
		if ws_name is None:
			ws_name = service_name
		if ws_name.startswith('/'):
			ws_name = ws_name[1:]
		
		self.services[ws_name] = Service(service_name,service_type)