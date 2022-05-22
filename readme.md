# Send Constant PWM to Blue Rov
A subscriber has been defined in listerner.py to listen to the topic do/thing. It expects a Int16 PWM value from the terminal and overrides the RC using setOverrideRCIN().

```
rostopic pub do/thing std_msgs/Int16 "data: 1560"
```