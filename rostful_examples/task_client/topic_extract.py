from __future__ import absolute_import
import os
import sys

import logging
import click

from rostful import rostful_celery_tasks

if __name__ == '__main__':
    result = rostful_celery_tasks.topic_extract.apply_async(
        args=['/turtle1/pose']
    )
    print result.get()  # waiting for result here ( but we don't have to )
    pass
