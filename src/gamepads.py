class JoystickRead:
    """ Assumes x-mode """
    def __init__(self, axes, buttons):
        self.axes = axes
        self.buttons = buttons

    """ Returns a value between 0 (not pressed) and 1 (pressed) """
    def get_left_trigger(self):
        raise NotImplementedError()
    
    """ Returns a value between 0 (not pressed) and 1 (pressed) """
    def get_right_trigger(self):
        raise NotImplementedError()

    """ Returns a value between -1 (left) and 1 (right) """
    def get_right_stick_x(self):
        raise NotImplementedError()
    

""" Represents a gamepad read for a logitech in X-mode """
class LogitechRead(JoystickRead):
    def __init__(self, axes, buttons):
        super().__init__(axes, buttons)

    def get_left_trigger(self):
        return 1 - (self.axes[2] + 1)/2

    def get_right_trigger(self):
        return 1 - (self.axes[5] + 1)/2

    def get_right_stick_x(self):
        return -self.axes[3]