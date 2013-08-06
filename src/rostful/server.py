import roslib
roslib.load_manifest('rostful')
import rospy
from rospy.service import ServiceManager
import rosservice, rostopic

from importlib import import_module
from collections import deque

import json
import sys
import re
from StringIO import StringIO

import message_conversion as msgconv
import deffile

ROS_MSG_MIMETYPE = 'application/vnd.ros.msg'
def ROS_MSG_MIMETYPE_WITH_TYPE(rostype):
	if isinstance(rostype,type):
		name = rostype.__name__
		module = rostype.__module__.split('.')[0]
		rostype = module + '/' + name
	return 'application/vnd.ros.msg; type=%s' % rostype

def get_json_bool(b):
	if b:
		return 'true'
	else:
		return 'false'

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
	
	def call(self,req,use_ros=False):
		rosreq = self.rostype_req()
		if use_ros:
			rosreq.deserialize(req)
		else:
			msgconv.populate_instance(req,rosreq)
		
		fields = []
		for slot in rosreq.__slots__:
			fields.append(getattr(rosreq,slot))
		fields = tuple(fields)
		
		rosresp = self.proxy(*fields)
		
		if use_ros:
			resp = StringIO()
			rosresp.serialize(resp)
			resp = resp.getvalue()
		else:
			resp = msgconv.extract_values(rosresp)
		return resp

class Topic:
	def __init__(self,topic_name,topic_type,allow_pub=True,allow_sub=True,queue_size=1):
		self.name = topic_name
		
		topic_type_module,topic_type_name = tuple(topic_type.split('/'))
		msg_module = import_module(topic_type_module + '.msg')
		
		self.rostype_name = topic_type
		self.rostype = getattr(msg_module,topic_type_name)
		
		self.allow_pub = allow_pub
		self.allow_sub = allow_sub
		
		self.msg = deque([],queue_size)
		
		self.pub = None
		if self.allow_pub:
			self.pub = rospy.Publisher(self.name, self.rostype)
		
		self.sub = None
		if self.allow_sub:
			self.sub = rospy.Subscriber(self.name, self.rostype, self.topic_callback)
	
	def publish(self,msg,use_ros=False):
		rosmsg = self.rostype()
		if use_ros:
			rosmsg.deserialize(msg)
		else:
			msgconv.populate_instance(msg,rosmsg)
		
		self.pub(rosmsg)
	
	def get(self,num=None,use_ros=False):
		if not self.msg:
			if use_ros:
				return ''
			else:
				return {}
		
		rosmsg = self.msg[0]
		if use_ros:
			msg = StringIO()
			rosmsg.serialize(msg)
			msg = msg.getvalue()
		else:
			msg = msgconv.extract_values(rosmsg)
		return msg
	
	def topic_callback(self,msg):
		self.msg.appendleft(msg)

CONFIG_PATH = '_rosdef'
SRV_PATH = '_srv'
MSG_PATH = '_msg'

def get_suffix(path):
	suffixes = '|'.join([re.escape(s) for s in [CONFIG_PATH,SRV_PATH,MSG_PATH]])
	match = re.search(r'/(%s)$' % suffixes, path)
	return match.group(1) if match else '' 

def response(start_response,status,data,content_type):
	content_length = 0
	if data is not None:
		content_length = len(data)
	headers = [('Content-Type', content_type),('Content-Length',str(content_length))]
	start_response(status, headers)
	return data

def response_200(start_response,data='',content_type='application/json'):
	return response(start_response,'200 OK',data,content_type)

def response_404(start_response,data='Invalid URL!',content_type='text/plain'):
	return response(start_response,'404 Not Found',data,content_type)

def response_405(start_response,data=[],content_type='text/plain'):
	return response(start_response,'405 Method Not Allowed',data,content_type)

def response_500(start_response,error,content_type='text/plain'):
	e_str = '%s: %s' % (str(type(error)),str(error))
	return response(start_response,'500 Internal Server Error',e_str,content_type)

class RostfulServer:
	def __init__(self):
		self.services = {}
		self.topics = {}
		
	def add_service(self,service_name,ws_name = None,service_type=None):
		resolved_service_name = rospy.resolve_name(service_name)
		if service_type is None:
			service_type = rosservice.get_service_type(resolved_service_name)
			
			if not service_type:
				print 'Unknown service %s' % service_name
				return False
		
		if ws_name is None:
			ws_name = service_name
		if ws_name.startswith('/'):
			ws_name = ws_name[1:]
		
		self.services[ws_name] = Service(service_name,service_type)
		return True
	
	def add_topic(self,topic_name,ws_name = None,topic_type=None,allow_pub=True,allow_sub=True):
		resolved_topic_name = rospy.resolve_name(topic_name)
		if topic_type is None:
			topic_type = rostopic.get_topic_type(resolved_topic_name)
		
		if ws_name is None:
			ws_name = topic_name
		if ws_name.startswith('/'):
			ws_name = ws_name[1:]
		
		self.topics[ws_name] = Topic(topic_name,topic_type,allow_pub=allow_pub,allow_sub=allow_sub)
	
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
	
	def _manifest(self,services, topics, indent = ''):
		dfile = deffile.DefFile()
		dfile.manifest.type = 'Node'
		
		service_section = deffile.INISection('Services')
		for service_name, service in services.iteritems():
			service_section.fields[service_name] = service.rostype_name
		dfile.add_section(service_section)
		
		topics_section = deffile.INISection('Topics')
		publish_section = deffile.INISection('Published')
		subscribe_section = deffile.INISection('Subscribed')
		
		for topic_name, topic in topics.iteritems():
			if topic.allow_pub and topic.allow_sub:
				topics_section[topic_name] = topic.rostype_name
			elif topic.allow_sub:
				publish_section[topic_name] = topic.rostype_name
			elif topic.allow_pub:
				subscribe_section[topic_name] = topic.rostype_name
		
		if topics_section.fields:
			dfile.add_section(topics_section)
		if publish_section.fields:
			dfile.add_section(publish_section)
		if subscribe_section.fields:
			dfile.add_section(subscribe_section)
		
		return dfile.tostring(suppress_formats=True)
		
		desc = ''
		desc += indent + 'Type: Manifest\n'
		
		desc += indent + 'Services:\n'
		for service_name, service in services.iteritems():
			desc += self._describe_service(service_name, service, indent=indent + '  ')
			desc += '\n'
		
		desc += indent + 'Topics:\n'
		for topic_name, topic in topics.iteritems():
			desc += self._describe_topic(topic_name, topic, indent=indent + '  ')
			desc += '\n'
		return desc[:-1]
	
	def _describe_service(self,service_name, service, indent = ''):
		dfile = deffile.DefFile()
		dfile.manifest.type = 'Service'
		dfile.manifest['Name'] = service_name
		dfile.manifest['Type'] = service.rostype_name
		
		return dfile.tostring(suppress_formats=True)
		
		desc = ''
		desc += indent + 'Path: %s\n' % service_name
		desc += indent + 'ROS_type: %s\n' % service.rostype_name
		return desc[:-1]
	
	def _describe_topic(self,topic_name, topic, indent = ''):
		dfile = deffile.DefFile()
		dfile.manifest.type = 'Topic'
		dfile.manifest['Name'] = topic_name
		dfile.manifest['Type'] = topic.rostype_name
		dfile.manifest['Published'] = get_json_bool(topic.allow_sub)
		dfile.manifest['Subscribed'] = get_json_bool(topic.allow_pub)
		
		return dfile.tostring(suppress_formats=True)
		
		desc = ''
		desc += indent + 'Path: %s\n' % topic_name
		desc += indent + 'ROS_type: %s\n' % topic.rostype_name
		desc += indent + 'Publish: %s\n' % get_json_bool(topic.allow_pub)
		desc += indent + 'Subscribe: %s\n' % get_json_bool(topic.allow_sub)
		return desc[:-1]
	
	def _handle_get(self,environ,start_response):
		path = environ['PATH_INFO']
		
		use_ros = environ.get('HTTP_ACCEPT','').find(ROS_MSG_MIMETYPE) != -1
		
		suffix = get_suffix(path)
		
		if not suffix:
			topic_name =  environ['PATH_INFO'][1:]
			if not self.topics.has_key(topic_name):
				return response_404(start_response)
			topic = self.topics[topic_name]
			
			if not topic.allow_sub:
				return response_405(start_response)
			
			msg = json.dumps(topic.get(use_ros=use_ros))
			
			if use_ros:
				content_type = ROS_MSG_MIMETYPE
			else:
				content_type = 'application/json'
			
			return response_200(start_response,msg,content_type=content_type)
			
		if path == '/' + CONFIG_PATH:
			config_data = self._manifest(self.services, self.topics)
			
			return response_200(start_response, config_data, content_type='text/plain')
		else:
			name = path[1:-(len(CONFIG_PATH)+1)]
			if self.services.has_key(name):
				service_name = name
				
				service = self.services[service_name]
				config_data = self._describe_service(service_name, service)
				
				return response_200(start_response, config_data, content_type='text/plain')
			elif self.topics.has_key(name):
				topic_name = name
				
				topic = self.topics[topic_name]
				config_data = self._describe_topic(topic_name, topic)
				
				return response_200(start_response, config_data, content_type='text/plain')
			else:
				return response_404(start_response)
		
	def _handle_post(self,environ,start_response):
		name =  environ['PATH_INFO'][1:]
		if not self.services.has_key(name) and not self.topics.has_key(name):
			return response_404(start_response)
		
		#from test_ros.srv import *
		#rosreq = AddTwoIntsRequest(1,2)
		#req = msgconv.extract_values(rosreq)
		
		try:
			#print 'calling service ', service.name
			
			length = int(environ['CONTENT_LENGTH'])
			content_type = environ['CONTENT_TYPE'].split(';')[0].strip()
			use_ros = content_type == ROS_MSG_MIMETYPE
			
			if not use_ros:
				input_msg = json.loads(environ['wsgi.input'].read(length))
				input_msg.pop('_format',None)
			
			ret_msg = None
			if self.services.has_key(name):
				service = self.services[name]
				ret_msg = service.call(input_msg,use_ros=use_ros)
			elif self.topics.has_key(name):
				topic = self.topics[name]
				if not topic.allow_pub:
					return response_405(start_response)
				topic.publish(input_msg,use_ros=use_ros)
			else:
				return response_404(start_response)
			
			if use_ros:
				content_type = ROS_MSG_MIMETYPE
			else:
				ret_msg['_format'] = 'ros'
				msg = json.dumps(ret_msg)
				content_type = 'application/json'
			
			return response_200(start_response, msg, content_type=content_type)
		except Exception, e:
			print 'Exception occurred!', e
			return response_500(start_response, e)

import argparse
from wsgiref.simple_server import make_server

def servermain():
	rospy.init_node('server',anonymous=True,disable_signals=True)
	
	parser = argparse.ArgumentParser()
	
	parser.add_argument('services',nargs='+')
	
	parser.add_argument('--host',default='')
	parser.add_argument('-p','--port',type=int,default=8080)
	
	args = parser.parse_args(rospy.myargv()[1:])
	
	try:
		server = RostfulServer()
		
		for service in args.services:
			ret = server.add_service(service)
			if ret:
				print 'Added service %s' % service
		
		httpd = make_server(args.host, args.port, server.wsgifunc())
		print 'Started server on port %d' % args.port
		
		#Wait forever for incoming http requests
		httpd.serve_forever()
		
	except KeyboardInterrupt:
		print 'Shutting down the server'
		httpd.socket.close()
		rospy.signal_shutdown('Closing')