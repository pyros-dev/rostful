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
def topic_inject(self, topic_name, **kwargs):
    res = self.app.ros_node_client.inject(topic_name, **kwargs)
    return res

@celery.task(bind=True)
def topic_extract(self, topic_name):
    res = self.app.ros_node_client.extract(topic_name)
    return res


@celery.task(bind=True)
def service(self, service_name, **kwargs):
    res = self.app.ros_node_client.call(service_name, **kwargs)
    return res

@celery.task(bind=True, base=AbortableTask)
def action(self, action_name, **kwargs):

    #interfacing celery task with ros action. both are supposed to represent a long running async job.

    #NOTE : kwargs can contain multiple goals (?) or one action task is one goal... more goals mean more action tasks ?

    # because Actions ( interpreted from roslib.js ) and because we don't want to know about the action msg type here
    # use the task ID as a goal ID
    goalID = 'goal_' + str(self.request.id)

    if not self.is_aborted():  # to make sure we didnt abort before it starts...
        res = self.app.ros_node_client.action_goal(action_name, goalID, **kwargs)

        # get full goalID
        if 'goal_id' in res.keys():
            goalID = res['goal_id']

        polling_period = 2.0
        while not self.is_aborted() and (
            res and res.get('goal_status', {}) and res.get('goal_status', {}).get('status') in [0, 1, 6, 7]
        ):
            # watch goal and feedback
            #TODO : estimate progression ? what if multiple goals ?
            res = self.app.ros_node_client.action_goal(action_name, goalID)
            # if res had empty status action si finished => we need to break out
            if not res.get('goal_status', {}):
                break
            else:
                self.update_state(
                    state='FEEDBACK',
                    meta={'rostful_data': res,}
                )

                time.sleep(polling_period)

        #detect action end and match celery status
        if self.is_aborted() or (
            res and res.get('goal_status', {}) and res.get('goal_status', {}).get('status', {}) in [4, 5, 8]
        ):
            #TODO : set state and info properly
            return celery.states.FAILURE
        elif (
            res and res.get('goal_status', {}) and res.get('goal_status', {}).get('status', {}) in [2, 3]
        ):
            res = self.app.ros_node_client.action_result(action_name, goalID)
            return res

    return {}  # unhandled status ??


@celery.task(bind=True, base=AbortableTask)
def rocon_app(self, rapp_name, input_data):

    rospy.wait_for_service('~start_rapp')
    try:
        call_service = rospy.ServiceProxy('~start_rapp', rostful_node.srv.StartRapp)
        res_data = call_service(rapp_name, input_data)
        if res_data.started:


            time.sleep(42)

            call_service = rospy.ServiceProxy('~stop_rapp', rostful_node.srv.StopRapp)
            res_data = call_service()





        return {'data_json': res_data.data_json}
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

