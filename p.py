class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def compute(self, process_variable, dt):

        error = self.setpoint - process_variable


        P_out = self.Kp * error


        self.integral += error * dt
        I_out = self.Ki * self.integral


        derivative = (error - self.previous_error) / dt
        D_out = self.Kd * derivative


        output = P_out + I_out + D_out


        self.previous_error = error

        return output



pid_x = 100
pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.0, setpoint=pid_x)

output = pid.compute(80, 1)
print(output)