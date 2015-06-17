from __future__ import absolute_import

import time

from celery import Celery, Task
from flask_celery import single_instance  # flask not needed for this
from redis import Redis

import random

from celery.utils.log import get_task_logger

_logger = get_task_logger(__name__)

#import required ros modules
from rostful_node import RostfulNode


from config.default import *
from config.development import *

celery = Celery(broker=CELERY_BROKER_URL, backend=CELERY_RESULT_BACKEND)
# needed for importers to get correct settings
celery.config_from_object('config.default')
celery.config_from_object('config.development')
redis = Redis()

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


#### Declaring more arguments to grab parametes from ros args or command line directly.
from celery import bootsteps
from celery.bin import Option

celery.user_options['worker'].add(
    Option("--ros_args", action="store", dest="ros_args", default=None, help="Activate support of rapps")
)

g_ros_args = []

class RosArgs(bootsteps.StartStopStep):

    def __init__(self, worker, ros_args, **options):
        # store the ros config
        global g_ros_args
        g_ros_args = ros_args.split()

celery.steps['worker'].add(RosArgs)




class Vegebot(Task):
    abstract = True
    _node = None
    def __init__(self, *args, **kwargs):
        super(Vegebot, self).__init__(*args, **kwargs)
        global g_ros_args
        try:
            self.ros_node = RostfulNode(g_ros_args)
            self.ros_if = self.ros_node.ros_if
            self.rocon_if = self.ros_node.rocon_if
            # List all requirement for this overseer to be able to start
            pass
        except Exception, e:
            raise

    def _requester_feedback(self, request_set):
        '''
          Callback used to act on feedback coming from the scheduler request handler.

          @param request_set : a snapshot of the state of all requests from this requester
          @type concert_scheduler_requests.transition.RequestSet
        '''
        _logger.error("REQUESTER FEEDBACK : %r", request_set)

    def after_return(self, *args, **kwargs):
        print('DeliveryOverseer returned: {0!r}'.format(self.request))

@celery.task(base=Vegebot, bind=True)
def topic_inject(self, topic_name,msg):
    #TODO : dynamically match kwargs and msg content
    topic = self.ros_if.topics[topic_name]
    msg = topic.get()
    pass

@celery.task(bind=True)
def topic_extract(self, topic_name):
    topic = self.ros_if.topics[topic_name]
    msg = topic.get()
    pass

@celery.task(bind=True)
def service_request(self, service_name):
    #TODO : dynamically match kwargs and msg content
    pass

@celery.task(bind=True)
def action_goal(self, action_name, goal):
    #TODO : dynamically match kwargs and msg content
    pass

#TODO : rocon stuff ( RAPP )