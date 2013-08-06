import roslib
from rostful.deffile import DefFile
roslib.load_manifest('rostful')
import rospy

import urllib2, json

import yaml

from importlib import import_module

import message_conversion as msgconv
import deffile

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
		req['_format'] = 'ros'
		reqs = json.dumps(req)
		
		wsreq = urllib2.Request(self.url,data=reqs,headers = {'Content-Type': 'application/json'})
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
		data.pop('_format',None)
		
		rosresp = self.rostype_resp()
		msgconv.populate_instance(data,rosresp)
		
		return rosresp
		

class IndividualTopicProxy:
	def __init__(self,url,name,topic_type,pub=True,sub=True):
		self.url = url
		self.name = name
		
		self.pub = pub
		self.sub = sub
		
		topic_type_module,topic_type_name = tuple(topic_type.split('/'))
		msg_module = import_module(topic_type_module + '.msg')
		
		self.rostype = getattr(msg_module,topic_type_name)
		
		self.proxy = None #TODO: implement

class RostfulServiceProxy:
	def __init__(self,url,remap=False):
		if url.endswith('/'):
			url = url[:-1]
		self.url = url
		
		self.services = {}
		self.topics = {}
		
		config_url = self.url + '/_rosdef'
		
		req = urllib2.Request(config_url)
		
		try:
			res = urllib2.urlopen(req)
		except Exception, e:
			#TODO: flip out
			raise e
		
		if res.getcode() != 200:
			#TODO: flip out
			pass
		
		parser = deffile.DefFileParser()
		parser.add_default_section_parser(deffile.INISectionParser)
		
		dfile = parser.parse(res.read().strip())
		
		if dfile.manifest_type == 'Node':
			services = dfile.get_section('Services')
			if services:
				for service_name, service_type in services.fields.iteritems():
					self.setup_service(self.url + '/' + service_name, service_name, service_type, remap=remap)
					print 'Service: %s (%s)' % (service_name, service_type)
			
			topics = dfile.get_section('Topics')
			if topics:
				for topic_name, topic_type in topics.fields.iteritems():
					self.setup_topic(self.url + '/' + topic_name, topic_name, topic_type, remap=remap)
			
			published_topics = dfile.get_section('Published')
			if published_topics:
				for topic_name, topic_type in published_topics.fields.iteritems():
					self.setup_topic(self.url + '/' + topic_name, topic_name, topic_type, pub=True, remap=remap)
			
			subscribed_topics = dfile.get_section('Subscribed')
			if subscribed_topics:
				for topic_name, topic_type in subscribed_topics.fields.iteritems():
					self.setup_topic(self.url + '/' + topic_name, topic_name, topic_type, sub=True, remap=remap)
		elif dfile.manifest_type == 'Service':
			self.setup_service(self.url, dfile.manifest['Name'], dfile.manifest['Type'], remap=remap)
		elif dfile.manifest_type == 'Topic':
			pub = dfile.manifest['Subscribed'].lower() == 'true'
			sub = dfile.manifest['Published'].lower() == 'true'
			self.setup_service(self.url, dfile.manifest['Name'], dfile.manifest['Type'], pub=pub, sub=sub, remap=remap)
		return
		
		data = yaml.load(res.read().strip())
		
		if data['Type'] == 'Manifest':
			for service in data['Services']:
				self.setup_service(self.url + '/' + service['Path'],service,remap)
		elif data['Type'] == 'Service':
			self.setup_service(self.url,data,remap)
		elif data['Type'] == 'Topic':
			self.setup_topic(self.url,data,remap)
		else:
			#TODO: flip out
			pass
	
	def setup_service(self, service_url, service_name, service_type, remap=False):
		if remap:
			service_name = service_name + '_ws'
		
		self.services[service_name] = IndividualServiceProxy(service_url,service_name,service_type)
	
	def setup_topic(self, topic_url, topic_name, topic_type, pub=None, sub=None, remap=False):
		if remap:
			topic_name = topic_name + '_ws'
		
		if pub is None and sub is None:
			pub = True
			sub = True
		
		self.topics[topic_name] = IndividualTopicProxy(topic_url,topic_name,topic_type,pub=sub,sub=sub)

import argparse

def proxymain():
	rospy.init_node('wsproxy',anonymous=True)
	
	parser = argparse.ArgumentParser()
	
	parser.add_argument('url')
	
	parser.add_argument('--test',action='store_true',default=False)
	
	args = parser.parse_args(rospy.myargv()[1:])
	
	if not args.url.startswith('http'):
		args.url = 'http://' + args.url
	
	proxy = RostfulServiceProxy(args.url, remap=args.test)
	
	rospy.spin()