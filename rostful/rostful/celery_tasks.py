from __future__ import absolute_import

import time

from celery import Task
from celery.contrib.abortable import AbortableTask
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

class ActionNotFound(Exception):
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

import unicodedata
import json
from rosinterface import message_conversion as msgconv
from rosinterface import ActionBack
import datetime

import rospy
import rostful_node
from importlib import import_module

@celery.task(bind=True)
def topic_inject(self, topic_name, input_data):
    rospy.wait_for_service('inject_topic')

    try:
        inject_topic = rospy.ServiceProxy('inject_topic', rostful_node.srv.InjectTopic)
        #TODO : check data format ?
        res = inject_topic(topic_name, input_data)
        return {'injected': res.injected}
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

@celery.task(bind=True)
def topic_extract(self, topic_name):

    rospy.wait_for_service('extract_topic')
    try:
        extract_topic = rospy.ServiceProxy('extract_topic', rostful_node.srv.ExtractTopic)
        res_data = extract_topic(topic_name)
        return {'extracted': res_data.extracted, 'data_json': res_data.data_json}
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

@celery.task(bind=True)
def service(self, service_name, input_data):

    rospy.wait_for_service('call_service')
    try:
        call_service = rospy.ServiceProxy('call_service', rostful_node.srv.CallService)
        res_data = call_service(service_name, input_data)
        return {'data_json': res_data.data_json}
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

@celery.task(bind=True, base=AbortableTask)
def action(self, action_name, input_data, feedback_time=2.0):

    # we want to wait for all services
    rospy.wait_for_service('start_action')
    rospy.wait_for_service('cancel_action')
    rospy.wait_for_service('status_action')

    # because Actions ( interpreted from roslib.js ) and because we don't wan to know about the action msg type here
    # use hte task ID as a goal ID
    goalID = 'goal_' + str(self.request.id)
    # Fill in the goal message
    msgdata = '{ "goal_id" : { "stamp" : { "secs" : 0, "nsecs" : 0 }, "id" : "'\
              + goalID\
              + '" }, "goal" : '\
              + input_data\
              + '}'

    started = False
    try:
        start_action = rospy.ServiceProxy('start_action', rostful_node.srv.StartAction)

        #TODO : check data format ?
        if not self.is_aborted():  # to make sure we didnt abort before it starts...
            res = start_action(action_name, msgdata)
            #goalID ??
            started = res.started

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    if started:

        try:
            cancel_action = rospy.ServiceProxy('cancel_action', rostful_node.srv.CancelAction)
            status_action = rospy.ServiceProxy('status_action', rostful_node.srv.StatusAction)
            feedback_action = rospy.ServiceProxy('feedback_action', rostful_node.srv.FeedbackAction)
            result_action = rospy.ServiceProxy('result_action', rostful_node.srv.ResultAction)

            while not self.is_aborted():
                #TODO We need to hook action statuses into the celery task statuses somehow...
                status = status_action(action_name)
                print "status %r", status
                feedback = feedback_action(action_name)
                print "feedback %r", feedback

                # if fb:  # if we get feedback, we expect the standard action msg definition
                #    self.update_state(state='FEEDBACK',
                #    meta={'header': fb.header, 'status': fb.status,
                #       'feedback': fb.feedback})

                time.sleep(feedback_time)

            if self.is_aborted():

                res = cancel_action(action_name, goalID)
                #goalID ??
            else:
                result = result_action(action_name)
                print "result %r", result
                return "{'header':" + result.header + ", 'status':" + result.status + ", 'feedback':" + result.feedback + "}"

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


@celery.task(bind=True, base=AbortableTask)
def rocon_app(self, rapp_name, input_data):

    rospy.wait_for_service('start_rapp')
    try:
        call_service = rospy.ServiceProxy('start_rapp', rostful_node.srv.StartRapp)
        res_data = call_service(rapp_name, input_data)
        if res_data.started:


            time.sleep(42)

            call_service = rospy.ServiceProxy('stop_rapp', rostful_node.srv.StopRapp)
            res_data = call_service()





        return {'data_json': res_data.data_json}
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

