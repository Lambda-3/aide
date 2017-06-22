#!/bin/bash
rosservice call /mario/add_rule "rule:
  name: 'genericDispatch'
  description: 'dispatches a command to the corresponding action provider/api'
  content: 'EXECUTE(util.dispatch_command) ?command [RANGE 5s STEP 1s] WHERE { ?s properties:command \"dispatch\". ?s properties:arg0 ?command} NEWONLY'"
rosservice call /mario/add_rule "rule:
  name: 'moveToPlace'
  description: 'Moves to a tagged place.'
  content: 'EXECUTE(move.move_to_place) ?name [RANGE 2s STEP 1s] WHERE {?s properties:command \"move_to\". ?s properties:arg0 ?name} NEWONLY'"
rosservice call /mario/add_rule "rule:
  name: 'tagPlace'
  description: 'tags a place'
  content: 'EXECUTE(map.tag_place) ?x ?y ?name [RANGE 5s STEP 1s] WHERE { mario:self properties:position_x ?x. mario:self properties:position_y ?y. ?s properties:command \"tag_place\". ?s properties:arg0 ?name} NEWONLY'"