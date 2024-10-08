## *********************************************************
##
## File autogenerated for the ugv_formation_config package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 245, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 290, 'description': 'int', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'int_param', 'edit_method': '', 'default': 50, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 290, 'description': 'double', 'max': 3.14, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'double_param', 'edit_method': '', 'default': 1.57, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 290, 'description': 'string', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'string_param', 'edit_method': '', 'default': 'hello world ', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 290, 'description': 'bool', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'bool_param', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 290, 'description': 'list', 'max': 2, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'list_param', 'edit_method': "{'enum_description': 'a car size set', 'enum': [{'srcline': 14, 'description': 'a small size', 'srcfile': '/home/fu/ugv_formation_control_ws/src/ugv_formation_config/cfg/main_config.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'small'}, {'srcline': 15, 'description': 'a medium size', 'srcfile': '/home/fu/ugv_formation_control_ws/src/ugv_formation_config/cfg/main_config.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'mediun'}, {'srcline': 16, 'description': 'a big size', 'srcfile': '/home/fu/ugv_formation_control_ws/src/ugv_formation_config/cfg/main_config.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'big'}]}", 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

dr_small = 0
dr_mediun = 1
dr_big = 2
