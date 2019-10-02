import threading
import control as ct
import frccontrol as fct
import numpy as np
from tkinter import *
from networktables import NetworkTables

cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

def drivetrain(motor, num_motors, m, r, rb, J, Gl, Gr):
    """Returns the state-space model for a drivetrain.
    States: [[left velocity], [right velocity]]
    Inputs: [[left voltage], [right voltage]]
    Outputs: [[left velocity], [right velocity]]
    Keyword arguments:
    motor -- instance of DcBrushedMotor
    num_motors -- number of motors driving the mechanism
    m -- mass of robot in kg
    r -- radius of wheels in meters
    rb -- radius of robot in meters
    J -- moment of inertia of the drivetrain in kg-m^2
    Gl -- gear ratio of left side of drivetrain
    Gr -- gear ratio of right side of drivetrain
    Returns:
    StateSpace instance containing continuous model
    """
    motor = fct.models.gearbox(motor, num_motors)

    C1 = -Gl ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -Gr ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C4 = Gr * motor.Kt / (motor.R * r)
    # fmt: off
    A = np.array([[(1 / m + rb**2 / J) * C1, (1 / m - rb**2 / J) * C3],
                  [(1 / m - rb**2 / J) * C1, (1 / m + rb**2 / J) * C3]])
    B = np.array([[(1 / m + rb**2 / J) * C2, (1 / m - rb**2 / J) * C4],
                  [(1 / m - rb**2 / J) * C2, (1 / m + rb**2 / J) * C4]])
    C = np.array([[1, 0],
                  [0, 1]])
    D = np.array([[0, 0],
                  [0, 0]])
    # fmt: on

    return ct.ss(A, B, C, D)
        
class ConstantsSender(fct.System):
    def __init__(self, root):
        NetworkTables.initialize(server='10.29.74.2')
        NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

        with cond:
            print("Waiting for NT connection")
            if not notified[0]:
                cond.wait()

        print("Connected!")
        
        self.sd = table = NetworkTables.getTable('SmartDashboard')
        self.root = root
        self.fields = 'Glow', 'Ghigh', 'Mass', 'Wheel radius', 'Robot radius', 'MOI'
        self.entries = []
        self.in_low_gear = True

        for field in self.fields:
            var = DoubleVar()
            var.set(1.0)
            var.trace("w", self.callback)
            label = Label(text=field)
            entry = Entry(self.root, textvariable=var)
            label.pack()
            entry.pack()
            self.entries.append((field, entry, var))

        u_min = np.array([[-12.0], [-12.0]])
        u_max = np.array([[12.0], [12.0]])
        fct.System.__init__(self, u_min, u_max, 0.02, np.zeros((2, 1)), np.zeros((2, 1)))

    def create_model(self, states, inputs):
        self.in_low_gear = False

        # Number of motors per side
        num_motors = 2.0

        # High and low gear ratios of drivetrain
        Glow = self.entries[0][2].get()
        Ghigh = self.entries[1][2].get()

        # Drivetrain mass in kg
        m = self.entries[2][2].get()
        # Radius of wheels in meters
        r = self.entries[3][2].get()
        # Radius of robot in meters
        self.rb = self.entries[4][2].get()
        # Moment of inertia of the drivetrain in kg-m^2
        J = self.entries[5][2].get()

        # Gear ratios of left and right sides of drivetrain respectively
        if self.in_low_gear:
            Gl = Glow
            Gr = Glow
        else:
            Gl = Ghigh
            Gr = Ghigh

        return drivetrain(fct.models.MOTOR_CIM, num_motors, m, r, self.rb, J, Gl, Gr)

    def design_controller_observer(self):
        if self.in_low_gear:
            q_vel = 1.0
        else:
            q_vel = 0.95

        q = [q_vel, q_vel]
        r = [12.0, 12.0]
        self.design_lqr(q, r)

        self.sd.putNumberArray('LQRMatrix', np.asarray(self.K))
        self.sd.putNumberArray('KalmanMatrix', np.asarray(self.kalman_gain))

        self.design_two_state_feedforward()
        
        q_vel = 1.0
        r_vel = 0.01
        self.design_kalman_filter([q_vel, q_vel], [r_vel, r_vel])

    def callback(self, *args):
        for field, entry, var in self.entries:
            try:
                self.sd.putNumber(field, var.get())
            except:
                return

        u_min = np.array([[-12.0], [-12.0]])
        u_max = np.array([[12.0], [12.0]])
        fct.System.__init__(self, u_min, u_max, 0.02, np.zeros((2, 1)), np.zeros((2, 1)))
        
root = Tk()
ConstantsSender(root)
root.mainloop()
