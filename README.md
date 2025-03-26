# **Robopy**
### Description:
This is a MicroPython library designed to simplify Lego robot development by providing tools such as a built-in movement system with line following, grid movement and more.
It is easy to use and enables fast iteration during development. Currently, it only supports two-wheeled EV3 robots.

### How to Download/Install:
First, download the correct version for your setup. Currently, we only support EV3 robots.
Then, copy the robopy.py file into your project and enjoy! :)
You can also find an example in main.py showing how to set up the basics for your robot.

## Documentation:
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
This method will make the bot move on a grid. If you give it *move_vector* (200, 100) and *max_turn_rad* 50mm it will move 150mm forward then do a 90° turn to the right with radius of 50mm and the continue 50mm forward to arive at final destination. If *max_turn_rad* is not specified it will make it as big as possible

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
