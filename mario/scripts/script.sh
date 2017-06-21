rosservice call /mario/add_rule "rule:
  name: 'generic_dispatch'
  description: 'dispatch from input reader'
  content: 'EXECUTE(util.dispatch_command) ?command [RANGE 5s STEP 1s] WHERE { ?s properties:command ?command.}'"