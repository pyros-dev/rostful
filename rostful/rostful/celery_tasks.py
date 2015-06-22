from __future__ import absolute_import

import time

from celery import Task
from flask_celery import Celery, single_instance  # flask not needed for this

import random

celery = Celery()

from celery.utils.log import get_task_logger

_logger = get_task_logger(__name__)

#import required ros modules
from rostful_node import RostfulNode

class TopicNotFound(Exception):
    pass

class ServiceNotFound(Exception):
    pass

@celery.task(bind=True)
@single_instance
def add_together(a, b):
    _logger.info("Sleeping 7s")
    time.sleep(7)
    _logger.info("Adding %s + %s" % (a, b))
    return a + b

@celery.task(bind=True)
def long_task(self):
    """Background task that runs a long function with progress reports."""
    verb = ['Starting up', 'Booting', 'Repairing', 'Loading', 'Checking']
    adjective = ['master', 'radiant', 'silent', 'harmonic', 'fast']
    noun = ['solar array', 'particle reshaper', 'cosmic ray', 'orbiter', 'bit']
    message = ''
    total = random.randint(10, 50)
    for i in range(total):
        if not message or random.random() < 0.25:
            message = '{0} {1} {2}...'.format(random.choice(verb),
                                              random.choice(adjective),
                                              random.choice(noun))
        self.update_state(state='PROGRESS',
                          meta={'current': i, 'total': total,
                                'status': message})
        time.sleep(1)
    return {'current': 100, 'total': 100, 'status': 'Task completed! from rostful celery_worker package', 'result': 42}


import inspect


class RostfulTask(Task):
    abstract = True
    _node = None
    def __init__(self, *args, **kwargs):
        super(RostfulTask, self).__init__(*args, **kwargs)
        global g_ros_args
        print('Initialized RostfulTask called by %r', inspect.stack()[1][3])


    def _requester_feedback(self, request_set):
        '''
          Callback used to act on feedback coming from the scheduler request handler.

          @param request_set : a snapshot of the state of all requests from this requester
          @type concert_scheduler_requests.transition.RequestSet
        '''
        _logger.error("REQUESTER FEEDBACK : %r", request_set)

    def after_return(self, *args, **kwargs):
        print('DeliveryOverseer returned: {0!r}'.format(self.request))

import unicodedata
import json
from rosinterface import message_conversion as msgconv
@celery.task(bind=True)
def topic_inject(self, topic_name, input_data):
    #TODO : dynamically match kwargs and msg content => maybe in topicBack instead ?
    topic_name = unicodedata.normalize('NFKD', topic_name).encode('ascii', 'ignore')
    if topic_name[0] == '/':
        topic_name = topic_name[1:]

    if not topic_name in self.ros_node.ros_if.topics:
        raise TopicNotFound
    else:
        topic = self.ros_node.ros_if.topics[topic_name]
        input_msg_type = topic.rostype

        input_msg = input_msg_type()
        input_data = json.loads(input_data)
        input_data.pop('_format', None)
        msgconv.populate_instance(input_data, input_msg)

        topic.publish(input_msg)  # NOT WORKING ?

    return "{}"

@celery.task(bind=True)
def topic_extract(self, topic_name):
    topic_name = unicodedata.normalize('NFKD', topic_name).encode('ascii', 'ignore')
    if topic_name[0] == '/':
        topic_name = topic_name[1:]
    if not topic_name in self.ros_node.ros_if.topics:
        raise TopicNotFound
    else:
        topic = self.ros_node.ros_if.topics[topic_name]
        msg = topic.get()
        # converting to json ( TMP )
        # TODO : convert to python dict for simple serialization by celery
        output_data = msgconv.extract_values(msg) if msg is not None else None
        output_data = json.dumps(output_data)

    return output_data

@celery.task(bind=True)
def service_call(self, service_name, input_data):
    #TODO : dynamically match kwargs and msg content => maybe in serviceBack instead ?
    service_name = unicodedata.normalize('NFKD', service_name).encode('ascii', 'ignore')
    if service_name[0] == '/':
        service_name = service_name[1:]

    if not service_name in self.ros_node.ros_if.services:
        raise ServiceNotFound
    else:

        service = self.ros_node.ros_if.services[service_name]
        input_msg_type = service.rostype_req
        input_msg = input_msg_type()

        input_data = json.loads(input_data)
        input_data.pop('_format', None)
        msgconv.populate_instance(input_data, input_msg)
        ret_msg = service.call(input_msg)

        output_data = msgconv.extract_values(ret_msg)
        #output_data['_format'] = 'ros'
        output_data = json.dumps(output_data)

    return output_data

@celery.task(bind=True)
def action_goal(self, action_name, goal):
    #TODO : dynamically match kwargs and msg content
    pass

#TODO : rocon stuff ( RAPP )