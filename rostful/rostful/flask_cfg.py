import os
basedir = os.path.abspath(os.path.dirname(__file__))

from . import celery_cfg

class Default(object):
    DEBUG = False
    TESTING = False

    SESSION_TYPE = 'filesystem'
    SECRET_KEY = '\x930)\xd5\x1d\x93\x19\xd6\xbf\x15\xa5c\xd9r\t\x8b)TP-7\xaal\x8e9m\xb2w?\x17\x9b1'

    # using flask instance directory
    SQLALCHEMY_DATABASE_URI = 'sqlite:///' + os.path.join(basedir, '..', 'instance', 'rostful_auth.db')

    @property
    def celery(self):
        return celery_cfg.Default

class Development(Default):
    DEBUG = True
    SQLALCHEMY_ECHO = False

    @property
    def celery(self):
        return celery_cfg.Development


class Production(Default):

    @property
    def celery(self):
        return celery_cfg.Production


class Testing(Default):
    TESTING = True

    @property
    def celery(self):
        return celery_cfg.Testing

