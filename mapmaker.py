from enum import IntEnum
import time
from typing import Tuple
from collections import namedtuple
from math import cos, sin, radians


# Constants
constants_t = namedtuple("constants", ["W", "H", "friction", "gravity", "restitution"])
constants = constants_t(900, 600, .001, .2, .9)

point = namedtuple('Point', ['x', 'y'])
line = namedtuple('Line', ['start', 'end'])

def pointSub(a: point, b: point):
    return point(a.x - b.x, a.y - b.y)

def pointAdd(a: point, b: point):
    return point(a.x + b.x, a.y + b.y)

def pointSubX(a: point, b: float):
    return point(a.x - b, a.y)

def pointAddX(a: point, b: float):
    return point(a.x + b, a.y)

def pointSubY(a: point, b: float):
    return point(a.x, a.y - b)

def pointAddY(a: point, b: float):
    return point(a.x, a.y + b)



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

    # Force normal to point from line → circle
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

    lines   :list[line]  = []
    circles :list[Circle]= []

    screen = pygame.display.set_mode((constants.W, constants.H))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    circles.append(Circle(constants.W/2, constants.H/2))

    class partTypes(IntEnum):
        line   = 0
        circle = 1
        spring = 2

    selectedType = partTypes.line
    circleSize = 10
    
    def text(msgs: Tuple[str], startPos=point(0, 0), lineIncrement=25):
        for lineno, msg in enumerate(msgs):
            screen.blit(font.render(msg, 0, (255, 255, 255)), (startPos.x, startPos.y + (lineIncrement * lineno)))

    def lineNormals(l: line):
        dx = l.end.x - l.start.x
        dy = l.end.y - l.start.y

        if dx == 0 and dy == 0:
            return point(0, 0), point(0, 0)

        inv_len = (dx*dx + dy*dy) ** 0.5
        dx /= inv_len
        dy /= inv_len

        n1 = point(-dy, dx)
        n2 = point(dy, -dx)

        return n1, n2

    lineBegin = False
    lineStartPos = point(0, 0)
    lineEndPos = point(0, 0)

    cameraOffset = point(0, 0)
    cameraIncrement = 25

    lineThickness = 2
    lineColour = (255, 255, 255)
    normalColour = (255, 0, 0)
    circleColour = (255, 255, 255)

    debugView = False
    def render():
        for c in circles:
            pygame.draw.circle(screen, circleColour, (int(c.x) + cameraOffset.x, int(c.y) + cameraOffset.y), c.radii)

        for l in lines:
            pygame.draw.line(screen, lineColour,\
             (l.start.x + cameraOffset.x, l.start.y + cameraOffset.y),\
             (l.end.x   + cameraOffset.x, l.end.y   + cameraOffset.y),\
            lineThickness)

            if debugView:
                left, right = lineNormals(l)
                pygame.draw.line(screen, normalColour, (((l.start.x + l.end.x) / 2) + cameraOffset.x,\
                                                        ((l.start.y + l.end.y) / 2) + cameraOffset.y),\
                                                       (((l.start.x + l.end.x) / 2) + left.x + cameraOffset.x,\
                                                        ((l.start.y + l.end.y) / 2) + left.y + cameraOffset.y))
                pygame.draw.line(screen, normalColour, (((l.start.x + l.end.x) / 2) + cameraOffset.x,\
                                                        ((l.start.y + l.end.y) / 2) + cameraOffset.y),\
                                                       (((l.start.x + l.end.x) / 2) + right.x + cameraOffset.x,\
                                                        ((l.start.y + l.end.y) / 2) + right.y + cameraOffset.y))


        if lineBegin:
            pygame.draw.line(screen, lineColour, lineStartPos, lineEndPos)

    while True:
        screen.fill((20, 20, 30))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_1:
                    selectedType = partTypes.line
                if event.key == pygame.K_2:
                    selectedType = partTypes.circle
                if event.key == pygame.K_3:
                    selectedType = partTypes.spring
                if event.key == pygame.K_TAB:
                    debugView = not debugView
                
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: # On left click, make
                    if lineBegin:
                        lineBegin = False
                        lines.append(line(pointAdd(lineStartPos, cameraOffset), pointAdd(lineEndPos, cameraOffset)))
                    else:
                        hit = event.pos
                        if selectedType == partTypes.circle:
                            circles.append(Circle(hit[0], hit[1], circleSize, 0, 0))
                        if selectedType == partTypes.line:
                            lineBegin = True
                            lineStartPos = point(*(pygame.mouse.get_pos()))
                elif event.button == 3: # On right click, remove closest
                    closestItemType = partTypes.circle
                    closestItemIndex = None
                    closestDist = None
                    mousePos = point(*(pygame.mouse.get_pos()))
                    
                    for i, c in enumerate(circles):
                        d = distance(point(c.x, c.y), mousePos) 
                        if closestDist:
                            if d < closestDist:
                                closestDist = d
                                closestItemIndex = i
                        else:
                            closestDist = d

                    for i, l in enumerate(lines):
                        d = distance(point(((l.start.x + l.end.x) / 2), ((l.start.y + l.end.y) / 2)), mousePos)
                        if d < closestDist:
                            closestItemType = partTypes.line
                            closestDist = d
                            closestItemIndex = i
                    
                    if closestItemType == partTypes.circle:
                        del circles[closestItemIndex]
                    elif closestItemType == partTypes.line:
                        del lines[closestItemIndex]

            if event.type == pygame.MOUSEWHEEL and selectedType == partTypes.circle:
                circleSize += event.y
                        
        if lineBegin:
            lineEndPos = point(*(pygame.mouse.get_pos()))

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            cameraOffset =  pointSubY(cameraOffset, cameraIncrement)
        if keys[pygame.K_a]:
            cameraOffset =  pointSubX(cameraOffset, cameraIncrement)
        if keys[pygame.K_s]:
            cameraOffset =  pointAddY(cameraOffset, cameraIncrement)
        if keys[pygame.K_d]:
            cameraOffset =  pointAddX(cameraOffset, cameraIncrement)



        simulate(); simulate()
        render()

        text((
            f"FPS: {clock.get_fps()}",
            f"Current Part: {selectedType}; size: {circleSize}",
        ))
        pygame.display.flip()
        clock.tick(60)