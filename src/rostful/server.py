from __future__ import absolute_import

import roslib
roslib.load_manifest('rostful')
import rospy
from rospy.service import ServiceManager
import rosservice, rostopic
import actionlib_msgs.msg

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
	def __init__(self, service_name, service_type):
		self.name = service_name
		
		service_type_module, service_type_name = tuple(service_type.split('/'))
		roslib.load_manifest(service_type_module)
		srv_module = import_module(service_type_module + '.srv')
		
		self.rostype_name = service_type
		self.rostype = getattr(srv_module, service_type_name)
		self.rostype_req = getattr(srv_module, service_type_name + 'Request')
		self.rostype_resp = getattr(srv_module, service_type_name + 'Response')
		
		self.proxy = rospy.ServiceProxy(self.name, self.rostype)
	
	def call(self, rosreq):
#		rosreq = self.rostype_req()
#		if use_ros:
#			rosreq.deserialize(req)
#		else:
#			msgconv.populate_instance(req, rosreq)
		
		fields = []
		for slot in rosreq.__slots__:
			fields.append(getattr(rosreq, slot))
		fields = tuple(fields)
		
		return self.proxy(*fields)

class Topic:
	def __init__(self, topic_name, topic_type, allow_pub=True, allow_sub=True, queue_size=1):
		self.name = topic_name
		
		topic_type_module, topic_type_name = tuple(topic_type.split('/'))
		roslib.load_manifest(topic_type_module)
		msg_module = import_module(topic_type_module + '.msg')
		
		self.rostype_name = topic_type
		self.rostype = getattr(msg_module, topic_type_name)
		
		self.allow_pub = allow_pub
		self.allow_sub = allow_sub
		
		self.msg = deque([], queue_size)
		
		self.pub = None
		if self.allow_pub:
			self.pub = rospy.Publisher(self.name, self.rostype)
		
		self.sub = None
		if self.allow_sub:
			self.sub = rospy.Subscriber(self.name, self.rostype, self.topic_callback)
	
	def publish(self, msg):
		self.pub.publish(msg)
		return
	
	def get(self, num=None):
		if not self.msg:
			return None
		
		return self.msg[0]
	
	def topic_callback(self, msg):
		self.msg.appendleft(msg)

"""
Publications: 
 * /averaging/status [actionlib_msgs/GoalStatusArray]
 * /averaging/result [actionlib_tutorials/AveragingActionResult]
 * /rosout [rosgraph_msgs/Log]
 * /averaging/feedback [actionlib_tutorials/AveragingActionFeedback]

Subscriptions: 
 * /random_number [unknown type]
 * /averaging/goal [actionlib_tutorials/AveragingActionGoal]
 * /averaging/cancel [actionlib_msgs/GoalID]
 """


class Action:
	STATUS_SUFFIX = 'status'
	RESULT_SUFFIX = 'result'
	FEEDBACK_SUFFIX = 'feedback'
	GOAL_SUFFIX = 'goal'
	CANCEL_SUFFIX = 'cancel'
	
	def __init__(self, action_name, action_type, queue_size=1):
		self.name = action_name
		
		action_type_module, action_type_name = tuple(action_type.split('/'))
		roslib.load_manifest(action_type_module)
		msg_module = import_module(action_type_module + '.msg')
		
		self.rostype_name = action_type
		
		self.rostype_action = getattr(msg_module, action_type_name + 'Action')
		
		self.rostype_action_goal = getattr(msg_module, action_type_name + 'ActionGoal')
		self.rostype_action_result = getattr(msg_module, action_type_name + 'ActionResult')
		self.rostype_action_feedback = getattr(msg_module, action_type_name + 'ActionFeedback')
		
		self.rostype_goal = getattr(msg_module, action_type_name + 'Goal')
		self.rostype_result = getattr(msg_module, action_type_name + 'Result')
		self.rostype_feedback = getattr(msg_module, action_type_name + 'Feedback')
		
		self.status_msg = deque([], queue_size)
		self.status_sub = rospy.Subscriber(self.name + '/' +self.STATUS_SUFFIX, actionlib_msgs.msg.GoalStatusArray, self.status_callback)
		
		self.result_msg = deque([], queue_size)
		self.result_sub = rospy.Subscriber(self.name + '/' +self.RESULT_SUFFIX, self.rostype_action_result, self.result_callback)
		
		self.feedback_msg = deque([], queue_size)
		self.feedback_sub = rospy.Subscriber(self.name + '/' +self.FEEDBACK_SUFFIX, self.rostype_action_feedback, self.feedback_callback)
		
		self.goal_pub = rospy.Publisher(self.name + '/' +self.GOAL_SUFFIX, self.rostype_action_goal)
		self.cancel_pub = rospy.Publisher(self.name + '/' +self.CANCEL_SUFFIX, actionlib_msgs.msg.GoalID)
	
	def get_msg_type(self, suffix):
		if suffix == self.STATUS_SUFFIX:
			return actionlib_msgs.msg.GoalStatusArray
		elif suffix == self.RESULT_SUFFIX:
			return self.rostype_action_result
		elif suffix == self.FEEDBACK_SUFFIX:
			return self.rostype_action_feedback
		elif suffix == self.GOAL_SUFFIX:
			return self.rostype_action_goal
		elif suffix == self.CANCEL_SUFFIX:
			return actionlib_msgs.msg.GoalID
		else:
			return None
	
	def publish_goal(self, msg):
		self.goal_pub.publish(msg)
		return
	
	def publish_cancel(self, msg):
		self.cancel_pub.publish(msg)
		return
	
	def publish(self, suffix, msg):
		if suffix == self.GOAL_SUFFIX:
			self.publish_goal(msg)
		elif suffix == self.CANCEL_SUFFIX:
			self.publish_cancel(msg)
	
	def get_status(self, num=None):
		if not self.status_msg:
			return None
		
		return self.status_msg[0]
	
	def get_result(self, num=None):
		if not self.result_msg:
			return None
		
		return self.result_msg[0]
	
	def get_feedback(self, num=None):
		if not self.feedback_msg:
			return None
		
		return self.feedback_msg[0]
	
	def get(self, suffix, num=None):
		if suffix == self.STATUS_SUFFIX:
			return self.get_status(num=num)
		elif suffix == self.RESULT_SUFFIX:
			return self.get_result(num=num)
		elif suffix == self.FEEDBACK_SUFFIX:
			return self.get_feedback(num=num)
		else:
			return None
	
	def status_callback(self, msg):
		self.status_msg.appendleft(msg)
	
	def result_callback(self, msg):
		self.result_msg.appendleft(msg)
	
	def feedback_callback(self, msg):
		self.feedback_msg.appendleft(msg)

CONFIG_PATH = '_rosdef'
SRV_PATH = '_srv'
MSG_PATH = '_msg'
ACTION_PATH = '_action'

def get_suffix(path):
	suffixes = '|'.join([re.escape(s) for s in [CONFIG_PATH,SRV_PATH,MSG_PATH,ACTION_PATH]])
	match = re.search(r'/(%s)$' % suffixes, path)
	return match.group(1) if match else '' 

def response(start_response, status, data, content_type):
	content_length = 0
	if data is not None:
		content_length = len(data)
	headers = [('Content-Type', content_type), ('Content-Length', str(content_length))]
	start_response(status, headers)
	return data

def response_200(start_response, data='', content_type='application/json'):
	return response(start_response, '200 OK', data, content_type)

def response_404(start_response, data='Invalid URL!', content_type='text/plain'):
	return response(start_response, '404 Not Found', data, content_type)

def response_405(start_response, data=[], content_type='text/plain'):
	return response(start_response, '405 Method Not Allowed', data, content_type)

def response_500(start_response, error, content_type='text/plain'):
	e_str = '%s: %s' % (str(type(error)), str(error))
	return response(start_response, '500 Internal Server Error', e_str, content_type)

class RostfulServer:
	def __init__(self):
		self.services = {}
		self.topics = {}
		self.actions = {}
	
	def add_service(self, service_name, ws_name=None, service_type=None):
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
		
		self.services[ws_name] = Service(service_name, service_type)
		return True
	
	def add_services(self, service_names):
		if not service_names:
			return
		print "Adding services:"
		for service_name in service_names:
			ret = self.add_service(service_name)
			if ret: print service_name
	
	def add_topic(self, topic_name, ws_name=None, topic_type=None, allow_pub=True, allow_sub=True):
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
		
		self.topics[ws_name] = Topic(topic_name, topic_type, allow_pub=allow_pub, allow_sub=allow_sub)
		return True
	
	def add_topics(self, topic_names, allow_pub=True, allow_sub=True):
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
	
	def add_action(self, action_name, ws_name=None, action_type=None):
		if action_type is None:
			resolved_topic_name = rospy.resolve_name(action_name + '/result')
			topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
			if not topic_type:
				print 'Unknown action %s' % action_name
				return False
			action_type = topic_type[:-len('ActionResult')]
		
		if ws_name is None:
			ws_name = action_name
		if ws_name.startswith('/'):
			ws_name = ws_name[1:]
		
		self.actions[ws_name] = Action(action_name, action_type)
		return True
	
	def add_actions(self, action_names):
		if not action_names:
			return
		print "Adding actions:"
		for action_name in action_names:
			ret = self.add_action(action_name)
			if ret: print action_name
	
	def wsgifunc(self):
		"""Returns the WSGI-compatible function for this server."""
		return self._handle
	
	def _handle(self, environ, start_response):
		if environ['REQUEST_METHOD'] == 'GET':
			return self._handle_get(environ, start_response)
		elif environ['REQUEST_METHOD'] == 'POST':
			return self._handle_post(environ, start_response)
		else:
			#TODO: flip out
			pass
	
	def _manifest(self, services, topics, actions, indent = ''):
		dfile = deffile.DefFile()
		dfile.manifest.def_type = 'Node'
		
		if services:
			service_section = deffile.INISection('Services')
			for service_name, service in services.iteritems():
				service_section.fields[service_name] = service.rostype_name
			dfile.add_section(service_section)
		
		topics_section = deffile.INISection('Topics')
		publish_section = deffile.INISection('Publishes')
		subscribe_section = deffile.INISection('Subscribes')
		
		for topic_name, topic in topics.iteritems():
			if topic.allow_sub:
				publish_section.fields[topic_name] = topic.rostype_name
			if topic.allow_pub:
				subscribe_section.fields[topic_name] = topic.rostype_name
		
		if topics_section.fields:
			dfile.add_section(topics_section)
		if publish_section.fields:
			dfile.add_section(publish_section)
		if subscribe_section.fields:
			dfile.add_section(subscribe_section)
		
		if actions:
			action_section = deffile.INISection('Actions')
			for action_name, action in actions.iteritems():
				action_section.fields[action_name] = action.rostype_name
			dfile.add_section(action_section)
		return dfile.tostring(suppress_formats=True)
	
	def _describe_service(self, service_name, service, indent=''):
		dfile = deffile.DefFile()
		dfile.manifest.def_type = 'Service'
		dfile.manifest['Name'] = service_name
		dfile.manifest['Type'] = service.rostype_name
		
		return dfile.tostring(suppress_formats=True)
	
	def _describe_topic(self, topic_name, topic, indent=''):
		dfile = deffile.DefFile()
		dfile.manifest.def_type = 'Topic'
		dfile.manifest['Name'] = topic_name
		dfile.manifest['Type'] = topic.rostype_name
		dfile.manifest['Publishes'] = get_json_bool(topic.allow_sub)
		dfile.manifest['Subscribes'] = get_json_bool(topic.allow_pub)
		
		return dfile.tostring(suppress_formats=True)
	
	def _describe_action(self, action_name, action, indent=''):
		dfile = deffile.DefFile()
		dfile.manifest.def_type = 'Action'
		dfile.manifest['Name'] = action_name
		dfile.manifest['Type'] = action.rostype_name
		
		return dfile.tostring(suppress_formats=True)
	
	def _get_msg(self, msg):
		return '\n'.join(['%s %s' % line for line in zip(msg._slot_types, msg.__slots__)])
	
	def _get_topic_msg(self, topic):
		return self._get_msg(topic.rostype)
	
	def _get_service_srv(self, service):
		return '\n'.join([self._get_msg(service.rostype_req),
						'---',
						self._get_msg(service.rostype_resp)
						])
	
	def _get_action_action(self, action):
		return '\n'.join([self._get_msg(action.rostype_goal),
						'---',
						self._get_msg(action.rostype_result),
						'---',
						self._get_msg(action.rostype_feedback)
						])
		
	
	def _handle_get(self, environ, start_response):
		path = environ['PATH_INFO'][1:]
		
		use_ros = environ.get('HTTP_ACCEPT','').find(ROS_MSG_MIMETYPE) != -1
		
		suffix = get_suffix(path)
		
		if path == CONFIG_PATH:
			config_data = self._manifest(self.services, self.topics, self.actions)
			return response_200(start_response, config_data, content_type='text/plain')
		
		if not suffix:
			if not self.topics.has_key(path):
				for action_suffix in [Action.STATUS_SUFFIX,Action.RESULT_SUFFIX,Action.FEEDBACK_SUFFIX]:
					action_name = path[:-(len(action_suffix)+1)]
					if path.endswith('/' + action_suffix) and self.actions.has_key(action_name):
						action = self.actions[action_name]
						msg = action.get(action_suffix)
						break
				else:
					return response_404(start_response)
			else:
				topic = self.topics[path]
				
				if not topic.allow_sub:
					return response_405(start_response)
				
				msg = topic.get()
			
			if use_ros:
				content_type = ROS_MSG_MIMETYPE
				output_data = StringIO()
				if msg is not None:
					msg.serialize(output_data)
				output_data = output_data.getvalue()
			else:
				content_type = 'application/json'
				output_data = msgconv.extract_values(msg) if msg is not None else None
				output_data = json.dumps(output_data)
			
			return response_200(start_response, output_data, content_type=content_type)
		
		path = path[:-(len(suffix)+1)]
		
		if suffix == MSG_PATH and self.topics.has_key(path):
				return response_200(start_response, self._get_topic_msg(self.topics[path]), content_type='text/plain')
		elif suffix == SRV_PATH and self.services.has_key(path):
				return response_200(start_response, self._get_service_srv(self.services[path]), content_type='text/plain')
		elif suffix == ACTION_PATH and self.actions.has_key(path):
				return response_200(start_response, self._get_action_action(self.actions[path]), content_type='text/plain')
		elif suffix == CONFIG_PATH:
			if self.services.has_key(path):
				service_name = path
				
				service = self.services[service_name]
				config_data = self._describe_service(service_name, service)
				
				return response_200(start_response, config_data, content_type='text/plain')
			elif self.topics.has_key(path):
				topic_name = path
				
				topic = self.topics[topic_name]
				config_data = self._describe_topic(topic_name, topic)
				
				return response_200(start_response, config_data, content_type='text/plain')
			elif self.actions.has_key(path):
				action_name = path
				
				action = self.actions[action_name]
				config_data = self._describe_action(action_name, action)
				
				return response_200(start_response, config_data, content_type='text/plain')
			else:
				return response_404(start_response)
		else:
			return response_404(start_response)
		
	def _handle_post(self, environ, start_response):
		name =  environ['PATH_INFO'][1:]
		
		try:
			#print 'calling service ', service.name
			length = int(environ['CONTENT_LENGTH'])
			content_type = environ['CONTENT_TYPE'].split(';')[0].strip()
			use_ros = content_type == ROS_MSG_MIMETYPE
			
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
				for suffix in [Action.GOAL_SUFFIX,Action.CANCEL_SUFFIX]:
					action_name = name[:-(len(suffix)+1)]
					if name.endswith('/' + suffix) and self.actions.has_key(action_name):
						mode = 'action'
						action_mode = suffix
						action = self.actions[action_name]
						input_msg_type = action.get_msg_type(suffix)
						break
				else:
					return response_404(start_response)
			
			input_data = environ['wsgi.input'].read(length)
			
			input_msg = input_msg_type()
			if use_ros:
				input_msg.deserialize(input_data)
			else:
				input_data = json.loads(input_data)
				input_data.pop('_format', None)
				msgconv.populate_instance(input_data, input_msg)
			
			ret_msg = None
			if mode == 'service':
				ret_msg = service.call(input_msg)
			elif mode == 'topic':
				topic.publish(input_msg)
				return response_200(start_response, [], content_type='application/json')
			elif mode == 'action':
				action.publish(action_mode, input_msg)
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
			print 'An exception occurred!', e
			return response_500(start_response, e)

import argparse
from wsgiref.simple_server import make_server

def servermain():
	rospy.init_node('rostful_server', anonymous=True, disable_signals=True)
	
	parser = argparse.ArgumentParser()
	
	parser.add_argument('--services', '--srv', nargs='+', help='Services to advertise')
	parser.add_argument('--topics', nargs='+', help='Topics to both publish and subscribe')
	parser.add_argument('--publishes', '--pub', nargs='+', help='Topics to publish via web services')
	parser.add_argument('--subscribes', '--sub', nargs='+', help='Topics to allowing publishing to via web services')
	parser.add_argument('--actions', nargs='+', help='Actions to advertise')
	
	parser.add_argument('--host', default='')
	parser.add_argument('-p', '--port', type=int, default=8080)
	
	args = parser.parse_args(rospy.myargv()[1:])
	
	try:
		server = RostfulServer()
		
		server.add_services(args.services)
		server.add_topics(args.topics)
		server.add_topics(args.publishes, allow_pub=False)
		server.add_topics(args.subscribes, allow_sub=False)
		server.add_actions(args.actions)
		
		httpd = make_server(args.host, args.port, server.wsgifunc())
		print 'Started server on port %d' % args.port
		
		#Wait forever for incoming http requests
		httpd.serve_forever()
		
	except KeyboardInterrupt:
		print 'Shutting down the server'
		httpd.socket.close()
		rospy.signal_shutdown('Closing')