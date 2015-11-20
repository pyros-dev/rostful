from __future__ import absolute_import
import os
import sys

import logging
import click

from rostful import rostful_celery_tasks

if __name__ == '__main__':

    result = rostful_celery_tasks.topic_inject.apply_async(
        args=['/turtle1/cmd_vel'],
        kwargs={
            'linear': {"x": 2.0, "y": 0.0, "z": 0.0},
            'angular': {"x": 0.0, "y": 0.0, "z": 1.0}
        }
    )

    print result.get()  # waiting for result here ( but we don't have to )
    pass
