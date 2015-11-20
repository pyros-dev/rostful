from __future__ import absolute_import
import os
import sys

import logging
import click
import time

from rostful import rostful_celery_tasks

import random

if __name__ == '__main__':

    result = rostful_celery_tasks.rocon_app.apply_async(
        args=['/gocart/gopher_rapps/delivery'],
        kwargs={
            "data": 5
        }
    )

    # while result.state != 'FAILURE' and result.state != 'SUCCESS':
    #     time.sleep(1)
    #     if result.state == 'PENDING':
    #         # job did not start yet
    #         print "Pending..."
    #
    #     elif result.state != 'FAILURE':
    #         response = {
    #             'state': result.state,
    #             'header': result.info.get('current', ''),
    #             'feedback': result.info.get('feedback', ''),
    #             'status': result.info.get('status', '')
    #         }
    #         if 'result' in result.info:
    #             response['result'] = result.info['result']
    #     else:
    #         # something went wrong in the background job
    #         response = {
    #             'state': result.state,
    #             'status': str(result.info),  # this is the exception raised
    #         }



    print result.get()  # waiting for result here ( but we don't have to )
    pass
