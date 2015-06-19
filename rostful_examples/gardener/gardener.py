from __future__ import absolute_import

import time

from celery import Celery, Task
from flask_celery import single_instance  # flask not needed for this
from redis import Redis

import random

#NOT WORKING
#from config.default import *
CELERY_ACCEPT_CONTENT = ['application/json']
CELERY_TASK_SERIALIZER = 'json'
CELERY_RESULT_SERIALIZER = 'json'
#from config.development import *
REDIS_URL = 'redis://localhost:6379'
CELERY_BROKER_URL = REDIS_URL
CELERY_RESULT_BACKEND = REDIS_URL

celery = Celery(broker=CELERY_BROKER_URL, backend=CELERY_RESULT_BACKEND)
# needed for importers to get correct settings



redis = Redis()

### defining extra tasks to be run by the vegebot
from vegebot import tasks
import turtlesim.msg

#locally define extended tasks ?
#@celery.tasks(base=tasks.Vegebot, bind=True)
#def move_turtle_via_topics(pose):
#    tasks.topic_inject('pose', Pose_msg)

### test methods

# testing turtle topics
def test_turtle_topics():
    # link topic extraction and topic inject to move relatively one turtle.
    result = tasks.topic_extract.apply_async(('pose', ))
    print result.get()


# testing turtle services
def test_turtle_services():
    pass

# testing turtle action
def test_turtle_action():
    pass