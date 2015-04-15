from __future__ import absolute_import

from importlib import import_module
import re

ROS_MSG_MIMETYPE = 'application/vnd.ros.msg'
def ROS_MSG_MIMETYPE_WITH_TYPE(rostype):
    if isinstance(rostype,type):
        name = rostype.__name__
        module = rostype.__module__.split('.')[0]
        rostype = module + '/' + name
    return 'application/vnd.ros.msg; type=%s' % rostype

def get_json_bool(b):
    if b:
        return 'true'
    else:
        return 'false'

def get_query_bool(query_string, param_name):
    return re.search(r'(^|&)%s((=(true|1))|&|$)' % param_name,query_string,re.IGNORECASE)

def type_str(msg):
    return msg.__module__.split('.')[0] + '/' + msg.__name__

def load_type(msg_type_name):
    if msg_type_name.endswith('[]'):
        msg_type_name = msg_type_name[:-2]
    module_name, type_name = msg_type_name.split('/')
    msg_module = import_module(module_name + '.msg')
    if not hasattr(msg_module,type_name) and (type_name.endswith('Request') or type_name.endswith('Response')):
        msg_module = import_module(module_name + '.srv')
    if not hasattr(msg_module,type_name):
        raise TypeError('Unknown ROS msg %s' % msg_type_name)
    return getattr(msg_module,type_name)