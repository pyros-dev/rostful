from __future__ import absolute_import

from . import deffile
from .util import get_json_bool, type_str, load_type

def get_all_msg_types(msg, skip_this=False, type_set=None):
    if type_set is None:
        type_set = set()
    if not skip_this:
        if msg in type_set:
            return type_set
        type_set.add(msg)
    for slot_type in msg._slot_types:
        if '/' not in slot_type:
            continue
        type_set = get_all_msg_types(load_type(slot_type), type_set=type_set)
    return type_set

def get_msg_definitions(msg, skip_this=False):
    type_set = get_all_msg_types(msg, skip_this=skip_this)
    
    msg_dfns = []
    for msg_type in type_set:
        dfn = deffile.ROSStyleDefinition('msg',type_str(msg_type),['msg'])
        for field_name, field_type in zip(msg_type.__slots__, msg_type._slot_types):
            dfn.segment(0).append((field_name, field_type))
        msg_dfns.append(dfn)
    return msg_dfns

def get_definitions(services=[], topics=[], actions=[]):
    service_dfns = []
    action_dfns = []
    msg_dfns = []
    
    type_set = set()
    for service in services:
        dfn = deffile.ROSStyleDefinition('srv',service.rostype_name,['request', 'response'])
        for field_name, field_type in zip(service.rostype_req.__slots__, service.rostype_req._slot_types):
            dfn.segment(0).append((field_name, field_type))
            type_set = get_all_msg_types(service.rostype_req, skip_this=True, type_set=type_set)
        for field_name, field_type in zip(service.rostype_resp.__slots__, service.rostype_resp._slot_types):
            dfn.segment(1).append((field_name, field_type))
            type_set = get_all_msg_types(service.rostype_resp, skip_this=True, type_set=type_set)
        service_dfns.append(dfn)
    
    for action in actions:
        dfn = deffile.ROSStyleDefinition('action',action.rostype_name,['goal', 'result', 'feedback'])
        for field_name, field_type in zip(action.rostype_goal.__slots__, action.rostype_goal._slot_types):
            dfn.segment(0).append((field_name, field_type))
            type_set = get_all_msg_types(action.rostype_goal, skip_this=True, type_set=type_set)
        for field_name, field_type in zip(action.rostype_result.__slots__, action.rostype_result._slot_types):
            dfn.segment(1).append((field_name, field_type))
            type_set = get_all_msg_types(action.rostype_result, skip_this=True, type_set=type_set)
        for field_name, field_type in zip(action.rostype_feedback.__slots__, action.rostype_feedback._slot_types):
            dfn.segment(2).append((field_name, field_type))
            type_set = get_all_msg_types(action.rostype_feedback, skip_this=True, type_set=type_set)
        action_dfns.append(dfn)
    
    for topic in topics:
        type_set = get_all_msg_types(topic.rostype, type_set=type_set)
    
    for msg_type in type_set:
        dfn = deffile.ROSStyleDefinition('msg',type_str(msg_type),['msg'])
        for field_name, field_type in zip(msg_type.__slots__, msg_type._slot_types):
            dfn.segment(0).append((field_name, field_type))
        msg_dfns.append(dfn)
    
    return msg_dfns + service_dfns + action_dfns

def manifest(services, topics, actions, full=False):
    dfile = deffile.DefFile()
    dfile.manifest.def_type = 'Node'
    
    if services:
        service_section = deffile.INISection('Services')
        for service_name, service in services.iteritems():
            service_section.fields[service_name] = service.rostype_name
        dfile.add_section(service_section)
    
    if actions:
        action_section = deffile.INISection('Actions')
        for action_name, action in actions.iteritems():
            action_section.fields[action_name] = action.rostype_name
        dfile.add_section(action_section)
    
    topics_section = deffile.INISection('Topics')
    publish_section = deffile.INISection('Publishes')
    subscribe_section = deffile.INISection('Subscribes')
    
    for topic_name, topic in topics.iteritems():
        if topic.allow_sub:
            publish_section.fields[topic_name] = topic.rostype_name
        if topic.allow_pub:
            subscribe_section.fields[topic_name] = topic.rostype_name
    
    if topics_section.fields:
        dfile.add_section(topics_section)
    if publish_section.fields:
        dfile.add_section(publish_section)
    if subscribe_section.fields:
        dfile.add_section(subscribe_section)
    
    if full:
        dfns = get_definitions(services=services.itervalues(), topics=topics.itervalues(), actions=actions.itervalues())
        [dfile.add_definition(dfn) for dfn in dfns]
    
    return dfile

def describe_service(service_name, service, full=False):
    dfile = deffile.DefFile()
    dfile.manifest.def_type = 'Service'
    dfile.manifest['Name'] = service_name
    dfile.manifest['Type'] = service.rostype_name
    
    if full:
        dfns = get_definitions(services=[service])
        [dfile.add_definition(dfn) for dfn in dfns]
    
    return dfile

def describe_topic(topic_name, topic, full=False):
    dfile = deffile.DefFile()
    dfile.manifest.def_type = 'Topic'
    dfile.manifest['Name'] = topic_name
    dfile.manifest['Type'] = topic.rostype_name
    dfile.manifest['Publishes'] = get_json_bool(topic.allow_sub)
    dfile.manifest['Subscribes'] = get_json_bool(topic.allow_pub)
    
    if full:
        dfns = get_definitions(topics=[topic])
        [dfile.add_definition(dfn) for dfn in dfns]
    
    return dfile

def describe_action(action_name, action, full=False):
    dfile = deffile.DefFile()
    dfile.manifest.def_type = 'Action'
    dfile.manifest['Name'] = action_name
    dfile.manifest['Type'] = action.rostype_name
    
    if full:
        _, action_dfns, msg_dfns = get_definitions(actions=[action])
        [dfile.add_definition(dfn) for dfn in msg_dfns]
        [dfile.add_definition(dfn) for dfn in action_dfns]
    
    return dfile

def describe_action_topic(action_name, suffix, action, full=False):
    dfile = deffile.DefFile()
    dfile.manifest.def_type = 'Topic'
    dfile.manifest['Name'] = action_name + '/' + suffix
    
    msg_type = action.get_msg_type(suffix)
    dfile.manifest['Type'] = type_str(msg_type)
    if suffix in [action.STATUS_SUFFIX,action.RESULT_SUFFIX,action.FEEDBACK_SUFFIX]:
        pub = True
        sub = False
    else:
        pub = False
        sub = True
    
    dfile.manifest['Publishes'] = get_json_bool(pub)
    dfile.manifest['Subscribes'] = get_json_bool(sub)
    
    if full:
        dfns = get_msg_definitions(msg_type)
        [dfile.add_definition(dfn) for dfn in dfns]
    
    return dfile

def get_msg(msg):
    return '\n'.join(['%s %s' % line for line in zip(msg._slot_types, msg.__slots__)])

def get_topic_msg(topic):
    return get_msg(topic.rostype)

def get_service_srv(service):
    return '\n'.join([get_msg(service.rostype_req),
                    '---',
                    get_msg(service.rostype_resp)
                    ])

def get_action_action(action):
    return '\n'.join([get_msg(action.rostype_goal),
                    '---',
                    get_msg(action.rostype_result),
                    '---',
                    get_msg(action.rostype_feedback)
                    ])