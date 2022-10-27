from tkinter import *
import time

from vector2 import *

drag_coeff = 1E-5
gravity = vec2(0, -9.81) # m/s^2
dt = 0

########################
#       CAMERA         #
########################

class camera():
    def __init__(self, name, pos, zoom, state):
        self.name = name
        self.pos = pos
        self.zoom = zoom
        self.state = state

    def activate(self):
        self.state = "active"

    def deactivate(self):
        self.state = "standby"

    def set_pos(self, pos):
        self.pos = pos

    def set_zoom(self, zoom):
        self.zoom = zoom

    def move(self, movement):
        self.pos += movement

    def do_zoom(self, zoom):
        self.zoom *= zoom

    def get_state(self):
        return self.state

    def get_pos(self):
        return self.pos

    def get_zoom(self):
        return self.zoom

def get_active_cam():
    current_cam = None

    for cam in cameras:
        if cam.get_state() == "active":
            current_cam = cam
            break

    return cam

def move_current_cam_left(event=None):
    get_active_cam().move(vec2(-30 * get_active_cam().get_zoom(), 0))

def move_current_cam_right(event=None):
    get_active_cam().move(vec2(30 * get_active_cam().get_zoom(), 0))

def move_current_cam_up(event=None):
    get_active_cam().move(vec2(0, 30 * get_active_cam().get_zoom()))

def move_current_cam_down(event=None):
    get_active_cam().move(vec2(0, -30 * get_active_cam().get_zoom()))

def zoom_current_cam_out(event=None):
    get_active_cam().do_zoom(2)

def zoom_current_cam_in(event=None):
    get_active_cam().do_zoom(0.5)

########################
#       GROUND         #
########################

class ground():
    def __init__(self, height, color, elasticity, k):
        self.height = height
        self.color = color
        self.elasticity = elasticity
        self.k = k

    def get_height(self):
        return self.height

    def get_color(self):
        return self.color

    def apply_force(self, points):
        for p in points:
            # normal force
            if p.get_pos().y < self.height:
                p.apply_force(vec2(0, p.mass * p.vel.y * -1 * (self.elasticity + 1) / dt))
                p.apply_force(gravity * p.mass)
                p.pos.y = self.height

            # friction
            if p.get_pos().y <= self.height:
                p.apply_force(vec2(p.vel.x, 0) * p.mass*gravity.mag()*self.k)

########################
#       LINK           #
########################

class rigid_link():
    def __init__(self, name, p1, p2, color, k=1000):
        self.name = name
        self.p1 = p1
        self.p2 = p2
        self.dist = get_dist_between(p1, p2)
        # spring coefficient
        self.k = k
        self.color = color

    def get_k(self):
        return self.k

    def get_name(self):
        return self.name

    def get_color(self):
        return self.color

    def apply_force(self):
        if get_dist_between(self.p1, self.p2) > self.dist:
            self.p1.apply_force(self.p1.get_unit_vector_towards(self.p2) * self.k*abs(get_dist_between(self.p1, self.p2) - self.dist))
            self.p2.apply_force(self.p2.get_unit_vector_towards(self.p1) * self.k*abs(get_dist_between(self.p1, self.p2) - self.dist))
            
        elif get_dist_between(self.p1, self.p2) < self.dist:
            self.p1.apply_force(self.p1.get_unit_vector_towards(self.p2) * -self.k*abs(get_dist_between(self.p1, self.p2) - self.dist))
            self.p2.apply_force(self.p2.get_unit_vector_towards(self.p1) * -self.k*abs(get_dist_between(self.p1, self.p2) - self.dist))

    def get_midpoint(self):
        return (self.p1.get_pos() + self.p2.get_pos())/2

########################
#     POINT MASS       #
########################

class point():
    def __init__(self, name, pos, vel, color, mass=1, static=False):
        self.name = name
        self.pos = pos
        self.vel = vel
        self.accel = vec2()
        self.mass = mass
        self.static = static
        self.color = color

        self.limit_axis = None

    def get_name(self):
        return self.name
    def get_pos(self):
        return self.pos
    def get_vel(self):
        return self.vel
    def get_mass(self):
        return self.mass
    def get_color(self):
        return self.color

    def get_unit_vector_towards(self, p2):
        return (p2.pos - self.pos)/(p2.pos - self.pos).mag()
    
    def get_vector_towards(self, p2):
        if type(p2) is point:
            return p2.pos - self.pos
        else:
            return p2 - self.pos

    def clear_accel(self):
        # call this every tick to not have residual forces from
        # previous frame
        self.accel = vec2(0,0)

    def apply_force(self, force):
        self.accel += force/self.mass

    def apply_gravity(self):
        self.apply_force(gravity * self.mass)

    def apply_drag(self):
        self.apply_force((self.vel.normalized() * -1) * (self.vel.mag()**2) * drag_coeff)

    def update_vel(self):
        if not self.static:
            self.vel += self.accel * dt

        if self.limit_axis:
            self.vel = self.limit_axis * self.vel.dot(self.limit_axis)

    def update_pos(self):
        if not self.static:
            self.pos += self.vel * dt

    def set_limit_axis(self, vec):
        if vec == "x":
            self.limit_axis = vec2(1,0)
        elif vec == "y":
            self.limit_axis = vec2(0, 1)
        else:
            self.limit_axis = vec.normalized()

########################
#    CONSTANT FORCE    #
########################

class const_force():
    def __init__(self, name, point, force):
        self.name = name
        self.point = point
        self.force = force

    def apply(self):
        self.point.apply_force(self.force)

def get_dist_between(p1, p2):
    if (type(p1) is point) and (type(p2) is point):
        return (p1.pos - p2.pos).mag()
    elif (type(p1) is point) and not (type(p2) is point):
        return (p1.pos - p2).mag()
    elif not (type(p1) is point) and (type(p2) is point):
        return (p1 - p2.pos).mag()
    else:
        return (p1 - p2).mag()

def space2canvas(space_coords):
    current_cam = get_active_cam()
    
    canvas_x = ((space_coords.x - current_cam.get_pos().x)/current_cam.get_zoom() + 900/2)
    canvas_y = ((-space_coords.y + current_cam.get_pos().y)/current_cam.get_zoom() + 500/2)
    return vec2(canvas_x, canvas_y)

def canvas2space(canvas_coords):
    current_cam = get_active_cam()

    space_x = (canvas_coords.x - 900/2)*current_cam.get_zoom() + current_cam.get_pos().x
    space_y = -((canvas_coords.y - 500/2)*current_cam.get_zoom() - current_cam.get_pos().y)

    return vec2(space_x, space_y)

def sign(number):
    if number >= 0:
        return 1
    else:
        return -1

def clicked_on_canvas(event):
    x = canvas2space(vec2(event.x,0)).x
    y = canvas2space(vec2(0, event.y)).y
    
    if click_op.get() == "cp":
        create_point(x, y)

    elif click_op.get() == "dp":
        delete_point(x, y)

    elif click_op.get() == "cl":
        create_link(x, y)

    elif click_op.get() == "dl":
        delete_link(x, y)

    elif click_op.get() == "af":
        apply_force_with_mouse(x, y, "l")

    elif click_op.get() == "rf":
        delete_force(x, y)

    elif click_op.get() == "cm":
        adjust_com_buffer(x, y, "l")

def right_clicked_on_canvas(event):
    x = canvas2space(vec2(event.x,0)).x
    y = canvas2space(vec2(0, event.y)).y

    if click_op.get() == "af":
        apply_force_with_mouse(x, y, "r")

    elif click_op.get() == "cm":
        adjust_com_buffer(x, y, "r")

def adjust_com_buffer(x, y, click):
    global calc_com_buffer

    if click == "l":
        if not get_closest_point_to_coords(x, y) in calc_com_buffer:
            calc_com_buffer.append(get_closest_point_to_coords(x, y))
    elif click == "r":
        if not len(calc_com_buffer) <= 0 and get_closest_point_to_coords(x, y) in calc_com_buffer:
            calc_com_buffer.remove(get_closest_point_to_coords(x, y))

def calc_com():
    global calc_com_buffer
    
    com_x = 0
    com_y = 0
    com_mass = 0
    
    for p in calc_com_buffer:
        com_mass += p.get_mass()
        com_x += p.get_pos().x * p.get_mass()
        com_y += p.get_pos().y * p.get_mass()

    com_x = com_x / com_mass
    com_y = com_y / com_mass

    return (vec2(com_x, com_y), com_mass)

def apply_force_with_mouse(x, y, click):
    global force_buffer

    if click == "r":
        if not get_closest_point_to_coords(x, y) in force_buffer:
            force_buffer.append(get_closest_point_to_coords(x, y))
        else:
            force_buffer.remove(get_closest_point_to_coords(x, y))

    elif click == "l":
        for p in force_buffer:
            create_force(x, y, p)
            force_buffer = []

def create_force(x, y, point):
    global forces
    forces.append(const_force(name_field.get("1.0","end-1c"), point, point.get_vector_towards(vec2(x, y)) * 0.01))

def delete_force(x, y):
    global forces
    force_tbd = get_closest_force_to_coords(x, y)

    if force_tbd:
        forces.remove(force_tbd)
        del force_tbd

def create_link(x, y):
    global linking_buffer
    
    if len(linking_buffer) == 0:
        linking_buffer.append(get_closest_point_to_coords(x, y))
    elif len(linking_buffer) == 1:
        if not get_closest_point_to_coords(x, y) == linking_buffer[0]:
            linking_buffer.append(get_closest_point_to_coords(x, y))
            new_link = rigid_link(name_field.get("1.0","end-1c"), linking_buffer[0], linking_buffer[1], link_color_field.get("1.0","end-1c"), float(link_const_field.get("1.0","end-1c")))
            links.append(new_link)

        linking_buffer = []

def delete_link(x, y):
    link_tbd = get_closest_link_to_coords(x, y)

    if link_tbd:
        links.remove(link_tbd)
        del link_tbd

def toggle_pause():
    global dt
    if dt > 0:
        dt = 0
    else:
        dt = 0.0025

def get_closest_point_to_coords(x, y):
    result = None
    for p in points:
        if not result or (vec2(x, y) - p.get_pos()).mag() < (vec2(x, y) - result.get_pos()).mag():
            result = p

    return result

def get_closest_link_to_coords(x, y):
    result = None
    for l in links:
        if not result or (vec2(x, y) - l.get_midpoint()).mag() < (result.get_midpoint() - vec2(x, y)).mag():
            result = l

    return result

def get_closest_force_to_coords(x, y):
    result = None
    for f in forces:
        if not result or ((vec2(x, y) - (f.point.get_pos() + f.force * 100)).mag() < (vec2(x, y), (result.point.get_pos() + result.force * 100).mag())):
            result = f

    return result

def create_point(x, y):
    new_point = point(name_field.get("1.0","end-1c"), vec2(x, y), vec2(), "seagreen", float(point_mass_field.get("1.0","end-1c")), staticPoint.get())
    points.append(new_point)

def delete_point(x, y):
    point_tbd = get_closest_point_to_coords(x, y)

    if point_tbd:

        # if point is an end of a link, delete the link
        # as well
        for l in links:
            if l.p1 == point_tbd or l.p2 == point_tbd:
                links.remove(l)
                del l
        
        points.remove(point_tbd)
        del point_tbd

root = Tk()
root.title("Mechuilibria")
root.geometry("1150x600")

# label controls
labelsLabel = Label(root, text="Labels")
labelsLabel.grid(row=0, column=0)

pointLabels = IntVar()
linkLabels = IntVar()

staticPoint = IntVar()

pointLabelType = StringVar()
linkLabelType = StringVar()

pointLabelType.set("n")
linkLabelType.set("n")

pointsLabelCheck = Checkbutton(root, text="Points", variable=pointLabels)
pointsLabelCheck.grid(row=1, column=0)

point_label_type_name = Radiobutton(root, text="Names", value="n", var = pointLabelType)
point_label_type_mass = Radiobutton(root, text="Masses", value="m", var = pointLabelType)

point_label_type_name.grid(row=2, column=0)
point_label_type_mass.grid(row=3, column=0)

linkLabelCheck = Checkbutton(root, text="Links", variable=linkLabels)
linkLabelCheck.grid(row=4, column=0)

link_label_type_name = Radiobutton(root, text="Names", value="n", var = linkLabelType)
link_label_type_k = Radiobutton(root, text="Spring Consts.", value="k", var = linkLabelType)

link_label_type_name.grid(row=5, column=0)
link_label_type_k.grid(row=6, column=0)

# pause-resume
pauseResumeButton = Button(root, text="Pause/Resume", command=toggle_pause)
pauseResumeButton.grid(row=7, column=0)

tk_canvas = Canvas(root, width=900, height=500, bg="white")
tk_canvas.grid(row=0, column=1, rowspan=15, columnspan=5)

main_cam = camera("main_cam", vec2(100, 50), 1, "active")

# canvas click
click_op = StringVar(root, "cp")
click_op_cp = Radiobutton(root, text="Create Point", value="cp", var = click_op)
click_op_dp = Radiobutton(root, text="Delete Point", value="dp", var = click_op)
click_op_cl = Radiobutton(root, text="Create Link", value="cl", var = click_op)
click_op_dl = Radiobutton(root, text="Delete Link", value="dl", var = click_op)

click_op_af = Radiobutton(root, text="Apply Force", value="af", var = click_op)
click_op_rf = Radiobutton(root, text="Remove Force", value="rf", var = click_op)

click_op_cm = Radiobutton(root, text="Calc. CoM", value="cm", var = click_op)

click_op_label = Label(root, text="Mouse Click Operation")
click_op_label.grid(row=0, column=6)

click_op_cp.grid(row=1, column=6)
click_op_dp.grid(row=2, column=6)
click_op_cl.grid(row=3, column=6)
click_op_dl.grid(row=4, column=6)

click_op_af.grid(row=5, column=6)
click_op_rf.grid(row=6, column=6)

click_op_cm.grid(row=7, column=6)

instruction = StringVar()
instruction_field = Label(root, textvariable=instruction)
instruction_field.grid(row=8, column=6, rowspan=3, padx=10)

bottom_options_label = Label(root, text="Create Point/Link Options")
bottom_options_label.grid(row=16, column=1)

name_field_label = Label(root, text="Name")
name_field_label.grid(row=17, column=1)
name_field = Text(root, height=1, width=20)
name_field.grid(row=18, column=1)

point_mass_field_label = Label(root, text="Point Mass (kg)")
point_mass_field_label.grid(row=17, column=2)
point_mass_field = Text(root, height=1, width=20)
point_mass_field.grid(row=18, column=2)

point_static_field = Checkbutton(root, text="Static Point", variable=staticPoint)
point_static_field.grid(row=19, column=1)

link_const_field_label = Label(root, text="Link Spring Constant")
link_const_field_label.grid(row=17, column=3)
link_const_field = Text(root, height=1, width=20)
link_const_field.grid(row=18, column=3)

link_color_field_label = Label(root, text="Link Color")
link_color_field_label.grid(row=17, column=4)
link_color_field = Text(root, height=1, width=20)
link_color_field.grid(row=18, column=4)

tk_canvas.bind('<Button-1>', clicked_on_canvas)
tk_canvas.bind('<Button-3>', right_clicked_on_canvas)

# camera controls
root.bind("<Up>", move_current_cam_up)
root.bind("<Down>", move_current_cam_down)
root.bind("<Left>", move_current_cam_left)
root.bind("<Right>", move_current_cam_right)
root.bind("<Control_L>", zoom_current_cam_out)
root.bind("<Shift_L>", zoom_current_cam_in)

# crane
n0 = point("n0", vec2(-30,-100), vec2(), "seagreen", 1, static=True)
n1 = point("n1", vec2(30,-100), vec2(), "seagreen", 1, static=True)
n2 = point("n2", vec2(-30,0), vec2(), "seagreen", 1)
n3 = point("n3", vec2(30,-0), vec2(), "seagreen", 1)
n4 = point("n4", vec2(-30,100), vec2(), "seagreen", 1)
n5 = point("n5", vec2(30,100), vec2(), "seagreen", 1)
n6 = point("n6", vec2(-30,200), vec2(), "seagreen", 1)
n7 = point("n7", vec2(30,200), vec2(), "seagreen", 1)

t0 = point("t0", vec2(-150, 100), vec2(), "seagreen", 5)
t1 = point("t1", vec2(-150, 200), vec2(), "seagreen", 7.5)

z0 = point("z0", vec2(350, 200), vec2(), "seagreen", 1.5)
z1 = point("z1", vec2(450, 200), vec2(), "seagreen", 1)
z2 = point("z2", vec2(350, 160), vec2(), "seagreen", 1.5)
z3 = point("z3", vec2(350, 50), vec2(50,0), "seagreen", 0.05)

m0 = rigid_link("m0", n0, n1, "skyblue", 1000)
m1 = rigid_link("m1", n2, n3, "skyblue", 1000)
m2 = rigid_link("m2", n4, n5, "skyblue", 1000)
m3 = rigid_link("m3", n6, n7, "skyblue", 1000)
m4 = rigid_link("m4", n0, n2, "skyblue", 1000)
m5 = rigid_link("m5", n2, n4, "skyblue", 1000)
m6 = rigid_link("m6", n4, n6, "skyblue", 1000)
m7 = rigid_link("m7", n1, n3, "skyblue", 1000)
m8 = rigid_link("m8", n3, n5, "skyblue", 1000)
m9 = rigid_link("m9", n5, n7, "skyblue", 1000)
m10 = rigid_link("m10", n0, n3, "magenta4", 200)
m11 = rigid_link("m11", n2, n5, "hotpink", 5000)
m12 = rigid_link("m12", n4, n7, "magenta4", 200)

r0 = rigid_link("r0", t0, n4, "skyblue", 1000)
r1 = rigid_link("r1", t1, n6, "skyblue", 1000)
r2 = rigid_link("r2", t0, t1, "skyblue", 1000)
r3 = rigid_link("r3", t0, n6, "hotpink", 5000)

f0 = rigid_link("f0", n7, z0, "skyblue", 1000)
f1 = rigid_link("f1", n5, z0, "hotpink", 5000)
f2 = rigid_link("f2", z0, z1, "skyblue", 1000)
f3 = rigid_link("f3", n5, z2, "hotpink", 5000)
f4 = rigid_link("f4", z2, z0, "skyblue", 1000)
f5 = rigid_link("f5", z2, z1, "skyblue", 1000)
f6 = rigid_link("f6", z2, n7, "skyblue", 1000)
f7 = rigid_link("f7", z2, z3, "orange", 2)

# lists of "things"
cameras = [main_cam]

floor = ground(-100, "green", 0.5, 0.8)

points = [n0, n1, n2, n3, n4, n5, n6, n7,
          t0, t1,
          z0, z1, z2, z3]

links = [m0, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12,
         r0, r1, r2, r3,
         f0, f1, f2, f3, f4, f5, f6, f7]

forces = []

force_buffer = []

linking_buffer = []

calc_com_buffer = []

while True:

    if click_op.get() == "cp":
        instruction.set("Click to create point at\nmouse cursor position.\nSet name and mass in\ninput fields.")
    elif click_op.get() == "dp":
        instruction.set("Click to remove point \nclosest to mouse cursor.")
    elif click_op.get() == "cl":
        instruction.set("Click to select points\nto link. Set name and\nspring constant in\ninput fields.")
    elif click_op.get() == "dl":
        instruction.set("Click to remove link\nclosest to mouse cursor.")
    elif click_op.get() == "af":
        instruction.set("Right click to select\npoints to apply force to.\nLeft click to set the\nforce vector.")
    elif click_op.get() == "rf":
        instruction.set("Click to remove force\nclosest to mouse cursor.")
    elif click_op.get() == "cm":
        instruction.set("Left click to choose\nmasses to calculate\ncenter of mass. Right\nclick to remove mass.")

    if not dt == 0:
        floor.apply_force(points)
    tk_canvas.create_rectangle(-1000, space2canvas(vec2(0, floor.get_height())).y,
                                1000, 500,
                                fill=floor.get_color())

    for f in forces:
        tk_canvas.create_line(space2canvas(f.point.get_pos()).x, space2canvas(f.point.get_pos()).y,
                              space2canvas(vec2(f.point.get_pos().x+f.force.x*100, f.point.get_pos().y)).x, space2canvas(vec2(f.point.get_pos().x, f.point.get_pos().y + f.force.y * 100)).y,
                              fill="blue", arrow=LAST)
        f.apply()

    for p in force_buffer:
        tk_canvas.create_oval(space2canvas(p.get_pos()).x-5, space2canvas(p.get_pos()).y-5,
                              space2canvas(p.get_pos()).x+5, space2canvas(p.get_pos()).y+5,
                              fill="blue")

    for p in linking_buffer:
        tk_canvas.create_oval(space2canvas(p.get_pos()).x-5, space2canvas(p.get_pos()).y-5,
                              space2canvas(p.get_pos()).x+5, space2canvas(p.get_pos()).y+5,
                              fill="red")

    if len(calc_com_buffer):
        for p in calc_com_buffer:
            tk_canvas.create_oval(space2canvas(p.get_pos()).x-5, space2canvas(p.get_pos()).y-5,
                              space2canvas(p.get_pos()).x+5, space2canvas(p.get_pos()).y+5,
                              fill="#ffc100")
        
        com_pos, com_mass = calc_com()
        tk_canvas.create_line(space2canvas(com_pos).x-8, space2canvas(com_pos).y-8,
                              space2canvas(com_pos).x+8, space2canvas(com_pos).y+8,
                              fill="#ffc100")

        tk_canvas.create_line(space2canvas(com_pos).x-8, space2canvas(com_pos).y+8,
                              space2canvas(com_pos).x+8, space2canvas(com_pos).y-8,
                              fill="#ffc100")

    for link in links:
        if not dt == 0:
            link.apply_force()
        tk_canvas.create_line(space2canvas(link.p1.get_pos()).x, space2canvas(link.p1.get_pos()).y,
                              space2canvas(link.p2.get_pos()).x, space2canvas(link.p2.get_pos()).y,
                              fill=link.get_color())
    
    for p in points:
        tk_canvas.create_oval(space2canvas(p.get_pos()).x-1, space2canvas(p.get_pos()).y-1,
                              space2canvas(p.get_pos()).x+1, space2canvas(p.get_pos()).y+1,
                              fill=p.get_color())

        if not dt == 0:
            p.apply_gravity()                    
            p.apply_drag()
            p.update_vel()
            p.update_pos()

    if pointLabels.get():
        if pointLabelType.get() == "n":
            for p in points:
                tk_canvas.create_text(space2canvas(p.get_pos()).x-10, space2canvas(p.get_pos()).y-10,
                                      text=p.get_name())
        elif pointLabelType.get() == "m":
            for p in points:
                tk_canvas.create_text(space2canvas(p.get_pos()).x-10, space2canvas(p.get_pos()).y-10,
                                      text=str(p.get_mass()))

    if linkLabels.get():
        if linkLabelType.get() == "n":
            for l in links:
                tk_canvas.create_text((space2canvas(l.p1.get_pos()).x + space2canvas(l.p2.get_pos()).x)/2,
                                      (space2canvas(l.p1.get_pos()).y + space2canvas(l.p2.get_pos()).y)/2,
                                      text=l.get_name(), fill=l.get_color())
        elif linkLabelType.get() == "k":
            for l in links:
                tk_canvas.create_text((space2canvas(l.p1.get_pos()).x + space2canvas(l.p2.get_pos()).x)/2,
                                      (space2canvas(l.p1.get_pos()).y + space2canvas(l.p2.get_pos()).y)/2,
                                      text=str(l.get_k()), fill=l.get_color())
    
    root.update()
    tk_canvas.delete("all")
    
    for p in points:
        p.clear_accel()

root.mainloop()
