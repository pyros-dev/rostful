from __future__ import absolute_import
import os
import sys

import logging
import click
import time

from rostful import rostful_celery_tasks

import random

if __name__ == '__main__':

    result = rostful_celery_tasks.action.apply_async(
        args=['/turtle_shape_server'],
        kwargs={
            "edges": 5,
            "radius": 3
        }
    )
    try:
        while result.state != 'FAILURE' and result.state != 'SUCCESS':
            time.sleep(1)
            if result.state == 'PENDING':
                # job did not start yet
                print "Pending..."

            elif result.state != 'FAILURE':
                print result.state, result.info.get('rostful_data', '')
                if 'result' in result.info:
                    print "RESULT: {0}".format(result.info['result'])
            else:
                print result.state, result.info.get('rostful_data', '')


        print result.get()  # waiting for result here ( but we don't have to )
        pass

    except KeyboardInterrupt:
        result.abort()
        #TODO Keep checking state
