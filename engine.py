from collections import namedtuple
from math import cos, sin, radians

# Constants
constants_t = namedtuple("constants", ["W", "H", "friction", "gravity", "restitution"])
constants = constants_t(900, 600, .001, .2, .9)

point = namedtuple('Point', ['x', 'y'])
line = namedtuple('Line', ['start', 'end'])

lineThickness = 2
lineColour = (255, 255, 255)
circleColour = (255, 255, 255)

center = point(constants.W/2, constants.H/2)

def rotatepoint(p:point, angle:float, pivot=center) -> point:
    rad = radians(angle)
    return point(
        (p.x - pivot.x) * cos(rad) - (p.y - pivot.y) * sin(rad) + pivot.x,
        (p.x - pivot.x) * sin(rad) + (p.y - pivot.y) * cos(rad) + pivot.y
    )

lines :list[line]= []
circles :list= []

def dot(xy: point, xy1: point):
    return xy.x * xy1.x + xy.y * xy1.y

def distance(p1: point, p2: point):
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5

def collisionNormalFromLine(lineSeg: line, closest: point, circlePos: point) -> point:
    dx = lineSeg.end.x - lineSeg.start.x
    dy = lineSeg.end.y - lineSeg.start.y

    nx, ny = -dy, dx
    length = (nx*nx + ny*ny) ** 0.5
    if length == 0:
        return point(0, 0)

    nx /= length
    ny /= length

    # Force normal to point from line -> circle
    to_circle_x = circlePos.x - closest.x
    to_circle_y = circlePos.y - closest.y

    if nx * to_circle_x + ny * to_circle_y < 0:
        nx = -nx
        ny = -ny

    return point(nx, ny)

class Circle:
    def __init__(self, x=0, y=0, radius=15, vx=5, vy=0):
        self.x = x
        self.y = y
        self.radii = radius
        self.vx = vx
        self.vy = vy

    def collideWithLine(self, l: line):
        dx = l.end.x - l.start.x
        dy = l.end.y - l.start.y
        seg_len_sq = dx*dx + dy*dy
        if seg_len_sq == 0:
            return

        t = ((self.x - l.start.x)*dx + (self.y - l.start.y)*dy) / seg_len_sq
        t = max(0, min(1, t))

        closest = point(
            l.start.x + dx * t,
            l.start.y + dy * t
        )

        dist = distance(point(self.x, self.y), closest)
        if dist >= self.radii:
            return

        normal = collisionNormalFromLine(
            l,
            closest,
            point(self.x, self.y)
        )

        overlap = self.radii - dist
        self.x += normal.x * overlap
        self.y += normal.y * overlap

        vel = point(self.vx, self.vy)
        vn = dot(vel, normal)
        if vn >= 0:
            return

        tangent = point(-normal.y, normal.x)
        vt = dot(vel, tangent)

        self.vx = (-vn * constants.restitution) * normal.x + vt * (1 - constants.friction) * tangent.x
        self.vy = (-vn * constants.restitution) * normal.y + vt * (1 - constants.friction) * tangent.y

    def collideWithCircle(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        dist_sq = dx*dx + dy*dy
        min_dist = self.radii + other.radii

        if dist_sq == 0 or dist_sq >= min_dist * min_dist:
            return

        dist_val = dist_sq ** 0.5
        nx = dx / dist_val
        ny = dy / dist_val

        overlap = min_dist - dist_val
        correction = overlap * 0.5
        self.x += nx * correction
        self.y += ny * correction
        other.x -= nx * correction
        other.y -= ny * correction

        rvx = self.vx - other.vx
        rvy = self.vy - other.vy
        vel_along_normal = rvx * nx + rvy * ny

        if vel_along_normal > 0:
            return

        j = -(1 + constants.restitution) * vel_along_normal * 0.5

        impulse_x = j * nx
        impulse_y = j * ny

        self.vx += impulse_x
        self.vy += impulse_y
        other.vx -= impulse_x
        other.vy -= impulse_y

    def simulate(self):
        self.vy += constants.gravity
        self.x += self.vx
        self.y += self.vy

        for l in lines:
            self.collideWithLine(l)

        for c in circles:
            if c is not self:
                self.collideWithCircle(c)

class Spring:
    def __init__(self, node1: Circle, node2: Circle, length: int = None, force: float = 1, thickness: float = 5):
        self.node1 = node1
        self.node2 = node2
        if (length is None): length = distance(point(node1.x - node2.x), point(node1.y - node2.y))
        self.length = length
        self.force = force
        self.thickness = thickness
    def simulate(self):
        distX = (self.node1.x + self.node1.vx*10) - (self.node2.x + self.node2.vx*10)
        distY = (self.node1.y + self.node1.vy*10) - (self.node2.y + self.node2.vy*10)
        distanceXY = distance(point(self.node1.x - self.node2.x), point(self.node1.y - self.node2.y))

        self.node1.vx += -distX * (1 - self.length/distanceXY)/2 * self.force
        self.node1.vy += -distY * (1 - self.length/distanceXY)/2 * self.force
        self.node2.vx +=  distX * (1 - self.length/distanceXY)/2 * self.force
        self.node2.vy +=  distX * (1 - self.length/distanceXY)/2 * self.force

def simulate():
    for c in circles:
        c.simulate()

if __name__ == "__main__":
    import pygame
    pygame.init()

    lines :list[line]= [
        line(point(constants.W/2 , 50            ), point(50            , constants.H/2 )),
        line(point(50            , constants.H/2 ), point(constants.W/2 , constants.H-50)),
        line(point(constants.W/2 , constants.H-50), point(constants.W-50, constants.H/2 )),
        line(point(constants.W-50, constants.H/2) , point(constants.W/2 , 50            ))
    ]

    screen = pygame.display.set_mode((constants.W, constants.H))
    clock = pygame.time.Clock()

    def render():
        for c in circles:
            pygame.draw.circle(screen, circleColour, (int(c.x), int(c.y)), c.radii)

        for l in lines:
            pygame.draw.line(screen, lineColour, (l.start.x, l.start.y), (l.end.x, l.end.y), lineThickness)

    circles.append(Circle(constants.W/2, constants.H/2))

    while True:
        screen.fill((20, 20, 30))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    circles.append(Circle(constants.W/2, constants.H/2))

        simulate(); simulate()
        render()


        for i, line_ in enumerate(lines):
            lines[i] = line(rotatepoint(line_.start, 1), rotatepoint(line_.end, 1))

        pygame.display.flip()
        clock.tick(60)
