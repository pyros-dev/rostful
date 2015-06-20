from __future__ import absolute_import

import time

from celery import Celery, Task
from flask_celery import single_instance  # flask not needed for this
from redis import Redis

import random

#NOT WORKING



# needed for importers to get correct settings



redis = Redis()

### defining extra tasks to be run by the vegebot
#from vegebot import tasks
import turtlesim.msg

#locally define extended tasks ?
#@celery.tasks(base=tasks.Vegebot, bind=True)
#def move_turtle_via_topics(pose):
#    tasks.topic_inject('pose', Pose_msg)

### test methods

from rostful import rostful_celery_tasks

# testing turtle topics
def test_turtle_topics():
    # link topic extraction and topic inject to move relatively one turtle.
    result = rostful_celery_tasks.topic_extract.apply_async(('/turtle1/pose', ))
    print result.get()
    pass

# testing turtle services
def test_turtle_services():
    pass

# testing turtle action
def test_turtle_action():
    pass