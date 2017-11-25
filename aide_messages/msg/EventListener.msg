# event types
uint8 Continuous=0
uint8 NewOnly=1
uint8 Precise=2
# unit types
string Millisecond = "ms"
string Second = "s"
string Minute = "m"
string Hour = "h"
string Day = "d"
# fields
string name
string[] params
uint8 range
string rangeUnit
uint8 step
string stepUnit
uint8 executionType
string sparqlWhere