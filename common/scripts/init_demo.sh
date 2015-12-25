#!/bin/bash

# add quadrotor instance
rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0
knowledge:
  knowledge_type: 0
  instance_type: 'quad'
  instance_name: 'q1'
  attribute_name: ''
  function_value: 0.0";

# add fact - quadrotor is on the ground
rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0
knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'grounded'
  values:
  - {key: 'q', value: 'q1'}
  function_value: 0.0";

# add goals - end on the ground and fly square
rosservice call /kcl_rosplan/update_knowledge_base "update_type: 1
knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'finished'
  values:
  - {key: 'q', value: 'q1'}
  function_value: 0.0";

rosservice call /kcl_rosplan/update_knowledge_base "update_type: 1
knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'squaredone'
  values:
  - {key: 'q', value: 'q1'}
  function_value: 0.0";

# call planner
rosservice call /kcl_rosplan/planning_server;
