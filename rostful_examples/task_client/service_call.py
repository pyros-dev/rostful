from __future__ import absolute_import
import os
import sys

import logging
import click

from rostful import rostful_celery_tasks

if __name__ == '__main__':

    result = rostful_celery_tasks.service.apply_async(
        args=['/turtle1/teleport_absolute'],
        kwargs={"y": 2, "x": 3, "theta": 2}
    )
    print result.get()  # waiting for result here ( but we don't have to )
    pass
