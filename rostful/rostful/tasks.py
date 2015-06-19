from __future__ import absolute_import

import time

from celery import Celery, Task
from flask_celery import single_instance  # flask not needed for this

import random

from celery.utils.log import get_task_logger

_logger = get_task_logger(__name__)

#import required ros modules
from rostful_node import RostfulNode

from .worker import rostful_worker


@rostful_worker.app.task(bind=True)
@single_instance
def add_together(a, b):
    _logger.info("Sleeping 7s")
    time.sleep(7)
    _logger.info("Adding %s + %s" % (a, b))
    return a + b

@rostful_worker.app.task(bind=True)
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



class Vegebot(Task):
    abstract = True
    _node = None
    def __init__(self, *args, **kwargs):
        super(Vegebot, self).__init__(*args, **kwargs)
        global g_ros_args


    def _requester_feedback(self, request_set):
        '''
          Callback used to act on feedback coming from the scheduler request handler.

          @param request_set : a snapshot of the state of all requests from this requester
          @type concert_scheduler_requests.transition.RequestSet
        '''
        _logger.error("REQUESTER FEEDBACK : %r", request_set)

    def after_return(self, *args, **kwargs):
        print('DeliveryOverseer returned: {0!r}'.format(self.request))

@rostful_worker.app.task(base=Vegebot, bind=True)
def topic_inject(self, topic_name,msg):
    #TODO : dynamically match kwargs and msg content
    topic = self.ros_if.topics[topic_name]
    msg = topic.get()
    pass

@rostful_worker.app.task(bind=True)
def topic_extract(self, topic_name):
    return
    topic = self.ros_if.topics[topic_name]
    msg = topic.get()
    pass

@rostful_worker.app.task(bind=True)
def service_request(self, service_name):
    #TODO : dynamically match kwargs and msg content
    pass

@rostful_worker.app.task(bind=True)
def action_goal(self, action_name, goal):
    #TODO : dynamically match kwargs and msg content
    pass

#TODO : rocon stuff ( RAPP )