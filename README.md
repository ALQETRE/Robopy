# **Robopy**
### Description:
This is a MicroPython library designed to simplify Lego robot development by providing tools such as a built-in movement system with line following, grid movement and more.
It is easy to use and enables fast iteration during development. Currently, it only supports two-wheeled EV3 robots.

### How to Download/Install:
First, download the correct version for your setup. Currently, we only support EV3 robots.
Then, copy the robopy.py file into your project and enjoy! :)
You can also find an example in main.py showing how to set up the basics for your robot.

# Documentation:
## class DriveBase:
#### class DriveBase(left_motor : Motor, right_motor : Motor, axle_len : int, ev3, optimal_battery_range : tuple = (0, 9))

| Name | Type | Desc |
|--|--|--|
| left_motor | robopy.Motor | Create a Motor object for left wheel |
| right_motor | robopy.Motor | Create a Motor object for right wheel |
| axle_len | int | Distance between centers of wheels in mm |
| ev3 | pybricks.hubs.EV3Brick() | EV3 Brick object |

### **Methods:**
---

#### battery_check(show : bool = False):

| Name | Type | Desc |
|--|--|--|
| show | bool | If true the voltage will be printed even if it is in range |

---

#### move(speed : float, dist : float):

| Name | Type | Desc |
|--|--|--|
| speed | float | The speed of the movement |
| dist | float | Distance to move forwards for in mm |

---

#### turn(speed : float, radius : float = 0, angle : float = 0):

| Name | Type | Desc |
|--|--|--|
| speed | float | The speed of the movement |
| radius | float | Radius of the turn in mm |
| angle | float | Angle of the arc |

---

#### grid_move(speed : float, move_vector : tuple = None, max_turn_rad : float = -1, rotation : float = 0):

##### Description:
This method will make the bot move on a grid. If you give it *move_vector* (200, 100) and *max_turn_rad* 50mm it will move 150mm forward then do a 90° arc turn to the right with radius of 50mm and the continue 50mm forward to arive at final destination. If *max_turn_rad* is not specified it will make it as big as possible

| Name | Type | Desc |
|--|--|--|
| speed | float | The speed of the movement |
| move_vector | tuple | Relative (X, Y) coordinates to move to in mm |
| max_turn_rad | float | Maximal turn radius the bot will make to get to it's destination in mm |
| rotation | float | The amount of degrees the bot will turn after its movement in degrees |

---

#### follow_line(speed : float, total_dist : float, line_follower : robopy.LineFollower = None):

##### Description:
This method will follow a line based on a *robopy.LineFollower*, you can give it as an argument and it will use it only once or you can use *set_follow_line()* to set the default. For more information see documentation on *set_line_follow()*

| Name | Type | Desc |
|--|--|--|
| speed | float | The speed of the movement |
| total_dist | float | The total distance it will cover in mm |
| line_follower | robopy.LineFollower | A line follower object that will be used only once |

---

#### set_follow_line(line_follower : LineFollower):

| Name | Type | Desc |
|--|--|--|
| line_follower | robopy.LineFollower | Give it a *robopy.LineFollower* and it will set it as default |

---

#### align(repeats : int, line_follower : robopy.LineFollower = None):

| Name | Type | Desc |
|--|--|--|
| repeats | int | Number of repeats to run follow_line |
| line_follower | robopy.LineFollower | A line follower object that will be used only once |

---

#### move_until(speed : float, until_func : function, args : list = None):

| Name | Type | Desc |
|--|--|--|
| speed | float | The speed of the movement |
| until_func | function | Expects function that returns boll and will move until it gets True |
| args | list | List of arguments for *until_func* |

---

#### turn_until(speed : float, until_func : function, radius : float = 0, direction : int = robopy.Right, args : list = None):

| Name | Type | Desc |
|--|--|--|
| speed | float | The speed of the movement |
| until_func | function | Expects function that returns boll and will move until it gets True |
| radius | float | Radius of the turn in mm |
| direction | robopy.Right/Left | Direction of arc |
| args | list | List of arguments for *until_func* |

---

#### follow_line_until(speed : float, until_func : function, line_follower : robopy.LineFollower = None, args : list = None):

| Name | Type | Desc |
|--|--|--|
| speed | float | The speed of the movement |
| until_func | function | Expects function that returns boll and will move until it gets True |
| line_follower | robopy.LineFollower | A line follower object that will be used only once |
| args | list | List of arguments for *until_func* |

---

#### calibrate_friction(move_func : function, friction : str):

##### Description:
This method moves the bot while measuring the distance traveld, then it asks you to correct it by rolling it back without slipping and then it calculates the over/undershoot and makes an multiplyer in the *friction.json*. **IMPORTANT** Make sure to download the friction.json onto your pc, because it will only update on the EV3 brick.

| Name | Type | Desc |
|--|--|--|
| move_func | function | Expects function that moves the bot while it tests the friction |
| friction | string | Name of the friction that is being calibrated |

---

#### set_friction(friction : string = "normal"):

| Name | Type | Desc |
|--|--|--|
| friction | string | Name of the friction that is going to be used from now on |

---
## class Motor:

#### class Motor(motor_port, motor_type : int = robopy.Drive, wheel_diameter : float = 0, ratio : float = 1, low_angle : float= 0, high_angle : float = 90):

| Name | Type | Desc |
|--|--|--|
| motor_port | pybricks.parameters.Port | The port where the motor is |
| motor_type | robopy.Drive/Actuator | Type of the motor |
| wheel_diameter | float | The diameter of the wheel in mm (If drive type) |
| ratio | float | Gear ratio |
| low_angle | float | The angle in degrees the actuator has on 0% travel (If actuator type) |
| high_angle | float | The angle in degrees the actuator has on 100% travel (If actuator type) |

### **Methods:**

---

#### set_drive(wheel_diameter, ratio):

| Name | Type | Desc |
|--|--|--|
| wheel_diameter | float | The diameter of the wheel in mm (If drive type) |
| ratio | float | Gear ratio |

---

#### run(dist : float, speed : float, wait : bool = False):

| Name | Type | Desc |
|--|--|--|
| dist | float | Distance to move forwards for in mm |
| speed | float | The speed of the movement |
| wait | bool | If true it waits on the movement to finnish |

---

#### run_free(speed : float):

| Name | Type | Desc |
|--|--|--|
| speed | float | The speed that it will run at until stoped |

---

#### free():

##### Description:
Reset's the total_angle of the motor and let's it spin freely

---

#### lock():

##### Description:
Locks the motor movement until moved by code

---

#### reset_angle():

##### Description:
Reset's the total_angle of the motor

---

#### get_movement():

##### Description:
Return's the total distance traveld

---

#### set_actuator(low_angle, high_angle, ratio):

| Name | Type | Desc |
|--|--|--|
| low_angle | float | The angle in degrees the actuator has on 0% travel (If actuator type) |
| high_angle | float | The angle in degrees the actuator has on 100% travel (If actuator type) |
| ratio | float | Gear ratio |

---

#### actuate(travel, speed):
| Name | Type | Desc |
|--|--|--|
| travel | float | Percentage of the angle range (0-100, can go higher or lower) |
| speed | float | The speed of the movement |

---

## class LineFollower:

#### LineFollower(eval_function : function, sensor : pybricks.ev3devices.*sensor_type* , target : float, side : int = Right, Kp : float = None, Ki : float = None, Kd : float = None):

| Name | Type | Desc |
|--|--|--|
| eval_function | function | Function that returns float between a negative and positive number *follow_line()* will try to make it *target* |
| sensor | pybricks.ev3devices.*sensor_type* | Sensor used for evaluating, it gets passed as a seceond parameter into eval_func |
| target | float | Target used for evaluating, it gets passed as a first parameter into eval_func |
| side | robopy.Left/Right | On what side of the line it should follow |
| Kp | float | Kp adjusts output based on the current error |
| Ki | float | Ki adjusts output based on past errors (Advanced) |
| Kd | float | Kd adjusts output based on the rate of error change |

---

## no class:

#### wait_for_button(ev3):

##### Description:
Waits on a button press and realese

| Name | Type | Desc |
|--|--|--|
| ev3 | pybricks.hubs.EV3Brick() | EV3 Brick object |

---

#### black_lf_eval(target, sensor):

##### Description:
Built-in line following evaluation function for following reflection of a color sensor

| Name | Type | Desc |
|--|--|--|
| target | float | The target for line following (lf) |
| sensor | float | The sensor for line following (lf) |
