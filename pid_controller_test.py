from simple_pid import PID

def increment(value):
    value


pid = PID(1, 0.1, 0.05, setpoint=10, sample_time=0.1)
v = 0
while True:
    control = pid(v)

    v = control
    print(v)
