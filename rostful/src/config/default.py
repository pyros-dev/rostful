import os
basedir = os.path.abspath(os.path.dirname(__file__))

DEBUG = False
TESTING = False

SESSION_TYPE = 'filesystem'
SECRET_KEY = '\x930)\xd5\x1d\x93\x19\xd6\xbf\x15\xa5c\xd9r\t\x8b)TP-7\xaal\x8e9m\xb2w?\x17\x9b1'

#TODO : use flask instance directory ?
SQLALCHEMY_DATABASE_URI = 'sqlite:///' + os.path.join(basedir, 'rostful_auth.db')
