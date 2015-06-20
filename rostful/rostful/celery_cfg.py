import os
basedir = os.path.abspath(os.path.dirname(__file__))

class Default(object):
    REDIS_URL = 'redis://localhost:6379'
    CELERY_BROKER_URL = REDIS_URL
    CELERY_RESULT_BACKEND = REDIS_URL
    CELERY_ACCEPT_CONTENT = ['application/json']
    CELERY_TASK_SERIALIZER = 'json'
    CELERY_RESULT_SERIALIZER = 'json'


class Development(Default):
    pass

class Production(Default):
    pass

class Testing(Default):
    pass
