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