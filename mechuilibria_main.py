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
    def __init__(self, pos=[0,0], vel=[0,0], mass=1, static=False):
        self.pos = pos
        self.vel = vel
        self.accel = [0,0]
        self.mass = mass
        self.static = static
        
    def get_pos(self):
        return self.pos
    def get_vel(self):
        return self.vel
    def get_mass(self):
        return self.mass

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

p0 = point(static=True)
p1 = point([10,-50], [0,0], 2)
p2 = point([-40,-60], [0,0], 3)

l0 = rigid_link(p0, p1, "skyblue", 1000)
l1 = rigid_link(p1, p2, "palevioletred", 1000)
l2 = rigid_link(p2, p0, "MistyRose4", 0.2)

points = [p0, p1, p2]
links = [l0, l1, l2]

while True:

    for link in links:
        link.apply_force()
        tk_canvas.create_line(space2canvas(link.p1.get_pos())[0], space2canvas(link.p1.get_pos())[1],
                              space2canvas(link.p2.get_pos())[0], space2canvas(link.p2.get_pos())[1],
                              fill=link.get_color())
    
    for p in points:
        p.apply_gravity()
        p.apply_drag()
        p.update_vel()
        p.update_pos()
    
    root.update()
    tk_canvas.delete("all")
    
    for p in points:
        p.clear_accel()

root.mainloop()


    
