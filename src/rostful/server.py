from __future__ import absolute_import

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

from . import message_conversion as msgconv
from . import deffile

from .util import ROS_MSG_MIMETYPE, get_json_bool

class Service:
	def __init__(self,service_name,service_type):
		self.name = service_name
		
		service_type_module,service_type_name = tuple(service_type.split('/'))
		roslib.load_manifest(service_type_module)
		srv_module = import_module(service_type_module + '.srv')
		
		self.rostype_name = service_type
		self.rostype = getattr(srv_module,service_type_name)
		self.rostype_req = getattr(srv_module,service_type_name + 'Request')
		self.rostype_resp = getattr(srv_module,service_type_name + 'Response')
		
		self.proxy = rospy.ServiceProxy(self.name, self.rostype)
	
	def call(self,rosreq):
#		rosreq = self.rostype_req()
#		if use_ros:
#			rosreq.deserialize(req)
#		else:
#			msgconv.populate_instance(req,rosreq)
		
		fields = []
		for slot in rosreq.__slots__:
			fields.append(getattr(rosreq,slot))
		fields = tuple(fields)
		
		return self.proxy(*fields)

class Topic:
	def __init__(self,topic_name,topic_type,allow_pub=True,allow_sub=True,queue_size=1):
		self.name = topic_name
		
		topic_type_module,topic_type_name = tuple(topic_type.split('/'))
		roslib.load_manifest(topic_type_module)
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
		
		self.pub.publish(rosmsg)
	
	def get(self,num=None,use_ros=False):
		if not self.msg:
			return None
		
		return self.msg[0]
		
	
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
	
	def add_services(self,service_names):
		if not service_names:
			return
		print "Adding services:"
		for service_name in service_names:
			ret = self.add_service(service_name)
			if ret: print service_name
	
	def add_topic(self,topic_name,ws_name = None,topic_type=None,allow_pub=True,allow_sub=True):
		resolved_topic_name = rospy.resolve_name(topic_name)
		if topic_type is None:
			topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
			if not topic_type:
				print 'Unknown topic %s' % topic_name
				return False
		
		if ws_name is None:
			ws_name = topic_name
		if ws_name.startswith('/'):
			ws_name = ws_name[1:]
		
		self.topics[ws_name] = Topic(topic_name,topic_type,allow_pub=allow_pub,allow_sub=allow_sub)
		return True
	
	def add_topics(self,topic_names,allow_pub=True,allow_sub=True):
		if not topic_names:
			return
		if allow_pub and allow_sub:
			print "Publishing and subscribing to topics:"
		elif allow_sub:
			print "Publishing topics:"
		elif allow_pub:
			print "Subscribing to topics"
		for topic_name in topic_names:
			ret = self.add_topic(topic_name, allow_pub=allow_pub, allow_sub=allow_sub)
			if ret:
				print topic_name
			
	
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
		publish_section = deffile.INISection('Publishes')
		subscribe_section = deffile.INISection('Subscribes')
		
		for topic_name, topic in topics.iteritems():
			if False and topic.allow_pub and topic.allow_sub:
				topics_section.fields[topic_name] = topic.rostype_name
			elif topic.allow_sub:
				publish_section.fields[topic_name] = topic.rostype_name
			elif topic.allow_pub:
				subscribe_section.fields[topic_name] = topic.rostype_name
		
		if topics_section.fields:
			dfile.add_section(topics_section)
		if publish_section.fields:
			dfile.add_section(publish_section)
		if subscribe_section.fields:
			dfile.add_section(subscribe_section)
		
		return dfile.tostring(suppress_formats=True)
	
	def _describe_service(self,service_name, service, indent = ''):
		dfile = deffile.DefFile()
		dfile.manifest.type = 'Service'
		dfile.manifest['Name'] = service_name
		dfile.manifest['Type'] = service.rostype_name
		
		return dfile.tostring(suppress_formats=True)
	
	def _describe_topic(self,topic_name, topic, indent = ''):
		dfile = deffile.DefFile()
		dfile.manifest.type = 'Topic'
		dfile.manifest['Name'] = topic_name
		dfile.manifest['Type'] = topic.rostype_name
		dfile.manifest['Published'] = get_json_bool(topic.allow_sub)
		dfile.manifest['Subscribed'] = get_json_bool(topic.allow_pub)
		
		return dfile.tostring(suppress_formats=True)
	
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
			
			msg = topic.get()
			
			if use_ros:
				content_type = ROS_MSG_MIMETYPE
				output_data = StringIO()
				msg.serialize(output_data)
				output_data = output_data.getvalue()
			else:
				content_type = 'application/json'
				output_data = msgconv.extract_values(msg)
				output_data = json.dumps(output_data)
			
			return response_200(start_response,output_data,content_type=content_type)
			
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
		
		try:
			#print 'calling service ', service.name
			length = int(environ['CONTENT_LENGTH'])
			content_type = environ['CONTENT_TYPE'].split(';')[0].strip()
			use_ros = content_type == ROS_MSG_MIMETYPE
			
			input_data = environ['wsgi.input'].read(length)
			
			if self.services.has_key(name):
				mode = 'service'
				service = self.services[name]
				input_msg_type = service.rostype_req
			elif self.topics.has_key(name):
				mode = 'topic'
				topic = self.topics[name]
				if not topic.allow_pub:
					return response_405(start_response)
				input_msg_type = topic.rostype
			else:
				return response_404(start_response)
			
			input_msg = input_msg_type()
			if use_ros:
				input_msg.deserialize(input_data)
			else:
				input_data = json.loads(input_data)
				input_data.pop('_format',None)
				msgconv.populate_instance(input_data,input_msg)
			
			ret_msg = None
			if mode == 'service':
				ret_msg = service.call(input_msg)
			else:
				topic.publish(input_msg,use_ros=use_ros)
				return response_200(start_response, [], content_type='application/json')
			
			if use_ros:
				content_type = ROS_MSG_MIMETYPE
				output_data = StringIO()
				ret_msg.serialize(output_data)
				output_data = output_data.getvalue()
			else:
				output_data = msgconv.extract_values(ret_msg)
				output_data['_format'] = 'ros'
				output_data = json.dumps(output_data)
				content_type = 'application/json'
			
			return response_200(start_response, output_data, content_type=content_type)
		except Exception, e:
			print 'Exception occurred!', e
			return response_500(start_response, e)

import argparse
from wsgiref.simple_server import make_server

def servermain():
	rospy.init_node('server',anonymous=True,disable_signals=True)
	
	parser = argparse.ArgumentParser()
	
	parser.add_argument('--services','--srv',nargs='+')
	parser.add_argument('--topics',nargs='+')
	parser.add_argument('--publishes','--pub',nargs='+')
	parser.add_argument('--subscribes','--sub',nargs='+')
	
	parser.add_argument('--host',default='')
	parser.add_argument('-p','--port',type=int,default=8080)
	
	args = parser.parse_args(rospy.myargv()[1:])
	
	try:
		server = RostfulServer()
		
		server.add_services(args.services)
		server.add_topics(args.topics)
		server.add_topics(args.publishes, allow_pub=False)
		server.add_topics(args.subscribes, allow_sub=False)
		
		httpd = make_server(args.host, args.port, server.wsgifunc())
		print 'Started server on port %d' % args.port
		
		#Wait forever for incoming http requests
		httpd.serve_forever()
		
	except KeyboardInterrupt:
		print 'Shutting down the server'
		httpd.socket.close()
		rospy.signal_shutdown('Closing')