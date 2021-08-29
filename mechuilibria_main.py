from tkinter import *
import time

drag_coeff = 0.005
gravity = -9.8 # m/s^2
dt = 0.002

def scale_vector(vector, scalar):
    result = []
    for element in vector:
        result.append(element * scalar)
    return result

def get_dist_between(p1, p2):
    return ((p1.pos[0] - p2.pos[0])**2 + (p1.pos[1] - p2.pos[1])**2)**0.5

class point():
    def __init__(self, name, pos, vel, color, mass=1, static=False):
        self.name = name
        self.pos = pos
        self.vel = vel
        self.accel = [0,0]
        self.mass = mass
        self.static = static
        self.color = color

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
        return [(p2.pos[0] - self.pos[0])/get_dist_between(self, p2),
                (p2.pos[1] - self.pos[1])/get_dist_between(self, p2)]
    def get_vector_towards(self, p2):
        return [(p2.pos[0] - self.pos[0]),
                (p2.pos[1] - self.pos[1])]

    def clear_accel(self):
        # call this every tick to not have residual forces from
        # previous frame
        self.accel = [0,0]

    def apply_force(self, force):
        self.accel[0] += force[0]/self.mass
        self.accel[1] += force[1]/self.mass

    def apply_gravity(self):
        self.accel[1] += gravity

    def apply_drag(self):
        self.apply_force([self.vel[0]**2 * drag_coeff * -sign(self.vel[0]),
                          self.vel[1]**2 * drag_coeff * -sign(self.vel[1])])

    def update_vel(self):
        if not self.static:
            self.vel[0] += self.accel[0] * dt
            self.vel[1] += self.accel[1] * dt

    def update_pos(self):
        if not self.static:
            self.pos[0] += self.vel[0] * dt
            self.pos[1] += self.vel[1] * dt

class rigid_link():
    def __init__(self, p1, p2, color, k=1000):
        self.p1 = p1
        self.p2 = p2
        self.dist = get_dist_between(p1, p2)
        # spring coefficient
        self.k = k
        self.color = color

    def get_color(self):
        return self.color

    def apply_force(self):
        if get_dist_between(self.p1, self.p2) > self.dist:
            self.p1.apply_force(scale_vector(self.p1.get_unit_vector_towards(self.p2),
                                             self.k*(get_dist_between(self.p1, self.p2) - self.dist)**2))
            self.p2.apply_force(scale_vector(self.p2.get_unit_vector_towards(self.p1),
                                             self.k*(get_dist_between(self.p1, self.p2) - self.dist)**2))
            
        elif get_dist_between(self.p1, self.p2) < self.dist:
            self.p1.apply_force(scale_vector(self.p1.get_unit_vector_towards(self.p2),
                                             -self.k*(get_dist_between(self.p1, self.p2) - self.dist)**2))
            self.p2.apply_force(scale_vector(self.p2.get_unit_vector_towards(self.p1),
                                             -self.k*(get_dist_between(self.p1, self.p2) - self.dist)**2))

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
            if p.get_pos()[1] < self.height:
                p.apply_force([0, p.mass * p.vel[1] * -1 * (self.elasticity + 1) / dt])
                p.pos[1] = self.height

            # friction
            if p.get_pos()[1] <= self.height:
                p.apply_force(scale_vector([p.vel[0], 0], p.mass*gravity*self.k))

def space2canvas(space_coords):
    canvas_x = space_coords[0] + 500/2
    canvas_y = -space_coords[1] + 500/2
    return [canvas_x, canvas_y]

def sign(number):
    if number >= 0:
        return 1
    else:
        return -1

root = Tk()
root.title("Mechuilibria")
root.geometry("500x500")

tk_canvas = Canvas(root, width=500, height=500, bg="white")
tk_canvas.pack(pady=20)

n0 = point("n0", [-30,-100], [0,0], "seagreen", 1)
n1 = point("n1", [30,-100], [0,0], "seagreen", 1)
n2 = point("n2", [-30,0], [0,0], "seagreen", 1)
n3 = point("n3", [30,-0], [0,0], "seagreen", 1)
n4 = point("n4", [-30,100], [500,0], "seagreen", 1)
n5 = point("n5", [30,100], [0,0], "seagreen", 1)
n6 = point("n6", [-30,200], [0,0], "seagreen", 1)
n7 = point("n7", [30,200], [0,0], "seagreen", 1)

m0 = rigid_link(n0, n1, "skyblue", 1000)
m1 = rigid_link(n2, n3, "skyblue", 1000)
m2 = rigid_link(n4, n5, "skyblue", 1000)
m3 = rigid_link(n6, n7, "skyblue", 1000)
m4 = rigid_link(n0, n2, "skyblue", 1000)
m5 = rigid_link(n2, n4, "skyblue", 1000)
m6 = rigid_link(n4, n6, "skyblue", 1000)
m7 = rigid_link(n1, n3, "skyblue", 1000)
m8 = rigid_link(n3, n5, "skyblue", 1000)
m9 = rigid_link(n5, n7, "skyblue", 1000)
m10 = rigid_link(n0, n3, "magenta4", 200)
m11 = rigid_link(n2, n5, "hotpink", 5000)
m12 = rigid_link(n4, n7, "magenta4", 200)

floor = ground(-100, "magenta4", 0.5, 0.2)

points = [n0, n1, n2, n3, n4, n5, n6, n7]
links = [m0, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12]
grounds = [floor]

labels = True

while True:

    floor.apply_force(points)
    tk_canvas.create_rectangle(0, space2canvas([0, floor.get_height()])[1],
                                500, 500,
                                fill=floor.get_color())

    for link in links:
        link.apply_force()
        tk_canvas.create_line(space2canvas(link.p1.get_pos())[0], space2canvas(link.p1.get_pos())[1],
                              space2canvas(link.p2.get_pos())[0], space2canvas(link.p2.get_pos())[1],
                              fill=link.get_color())
    
    for p in points:
        tk_canvas.create_oval(space2canvas(p.get_pos())[0]-1, space2canvas(p.get_pos())[1]-1,
                              space2canvas(p.get_pos())[0]+1, space2canvas(p.get_pos())[1]+1,
                              fill=p.get_color())

        if not p.pos[1] <= floor.get_height():
            p.apply_gravity()
                              
        p.apply_drag()
        p.update_vel()
        p.update_pos()

    if labels:
        for p in points:
            tk_canvas.create_text(space2canvas(p.get_pos())[0]-10, space2canvas(p.get_pos())[1]-10,
                                  text=p.get_name())
    
    root.update()
    tk_canvas.delete("all")
    
    for p in points:
        p.clear_accel()

root.mainloop()
