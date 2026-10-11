"""Choose a map-checked crossing with waiting points outside the swept track."""
import math


def track_distance(point, track):
    sx, sy, ex, ey = track['points']
    dx, dy = ex-sx, ey-sy
    t = max(0., min(1., ((point[0]-sx)*dx+(point[1]-sy)*dy)/(dx*dx+dy*dy)))
    return math.hypot(point[0]-sx-t*dx, point[1]-sy-t*dy)


def static_clearance(grid, point, limit=1.3):
    """Distance to static/unknown cells or the map boundary, in metres."""
    if grid is None or grid.info.resolution <= 0:
        return 0.
    origin = grid.info.origin
    q = origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = point[0]-origin.position.x, point[1]-origin.position.y
    x, y = c*dx+s*dy, -s*dx+c*dy
    r, width, height = grid.info.resolution, grid.info.width, grid.info.height
    if not (0 <= x < width*r and 0 <= y < height*r):
        return 0.
    nearest = min(limit, x, y, width*r-x, height*r-y)
    cell_margin = r/math.sqrt(2.)
    for gy in range(max(0, math.floor((y-limit)/r)),
                    min(height, math.ceil((y+limit)/r))):
        for gx in range(max(0, math.floor((x-limit)/r)),
                        min(width, math.ceil((x+limit)/r))):
            value = grid.data[gy*width+gx]
            if value < 0 or value >= 65:
                nearest = min(nearest, max(0., math.hypot(
                    (gx+.5)*r-x, (gy+.5)*r-y)-cell_margin))
    return nearest


def clear_segment(grid, first, second, radius=.45):
    if grid is None or grid.info.resolution <= 0:
        return False
    origin = grid.info.origin
    q = origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    length = math.dist(first, second)
    count = max(1, math.ceil(length/.04))
    offsets = [(0., 0.)] + [(r*math.cos(i*math.pi/4), r*math.sin(i*math.pi/4))
        for r in (.15, .30, radius) for i in range(8)]
    for tick in range(count+1):
        x = first[0]+tick/count*(second[0]-first[0])
        y = first[1]+tick/count*(second[1]-first[1])
        for dx, dy in offsets:
            wx, wy = x+dx-origin.position.x, y+dy-origin.position.y
            gx = math.floor((c*wx+s*wy)/grid.info.resolution)
            gy = math.floor((-s*wx+c*wy)/grid.info.resolution)
            if not (0 <= gx < grid.info.width and 0 <= gy < grid.info.height):
                return False
            value = grid.data[gy*grid.info.width+gx]
            if value < 0 or value >= 65:
                return False
    return True


def choose_passage(points, track, grid):
    if len(points) < 2:
        return None
    sx, sy, ex, ey = track['points']
    length = math.hypot(ex-sx, ey-sy)
    nx, ny = -(ey-sy)/length, (ex-sx)/length
    signed = lambda p: (p[0]-sx)*nx+(p[1]-sy)*ny
    if signed(points[0])*signed(points[-1]) >= 0:
        return None
    near = False
    for a, b in zip(points, points[1:]):
        count = max(1, math.ceil(math.dist(a, b)/.1))
        if any(track_distance((a[0]+i/count*(b[0]-a[0]),
                               a[1]+i/count*(b[1]-a[1])), track) < .95
               for i in range(count+1)):
            near = True; break
    if not near:
        return None
    side = 1 if signed(points[0]) > 0 else -1
    # If a permanent lane exists beyond either end, use it instead of forcing
    # a timed crossing. The 1.2 m centre offset includes footprint, box extent,
    # prediction margin and 0.15 m waypoint tolerance.
    bypasses = []
    ux, uy = (ex-sx)/length, (ey-sy)/length
    for x, y in ((sx-1.2*ux, sy-1.2*uy), (ex+1.2*ux, ey+1.2*uy)):
        entry = (x+side*1.3*nx, y+side*1.3*ny)
        exit = (x-side*1.3*nx, y-side*1.3*ny)
        if clear_segment(grid, entry, exit, radius=.6):
            bypasses.append((math.dist(points[0],entry)+math.dist(entry,exit)+math.dist(exit,points[-1]),entry,exit))
    # Axis-aligned lanes can fit the doorway better than a line normal to a
    # diagonal track. Check the entire swept bounding box plus the same margin.
    xmin, xmax, ymin, ymax = min(sx,ex), max(sx,ex), min(sy,ey), max(sy,ey)
    lanes = [((x,ymin-1.3),(x,ymax+1.3)) for x in (xmin-1.2,xmax+1.2)]
    lanes += [((xmin-1.3,y),(xmax+1.3,y)) for y in (ymin-1.2,ymax+1.2)]
    for a,b in lanes:
        if not clear_segment(grid,a,b,radius=.6):
            continue
        for entry,exit in ((a,b),(b,a)):
            bypasses.append((math.dist(points[0],entry)+math.dist(entry,exit)+math.dist(exit,points[-1]),entry,exit))
    if bypasses:
        _, entry, exit = min(bypasses)
        return dict(track=track, entry=entry, exit=exit, bypass=True,
                    yaw=math.atan2(exit[1]-entry[1],exit[0]-entry[0]))
    candidates = []
    # Waiting points leave room for Nav2's 0.15 m arrival tolerance and braking.
    offset = 1.30
    for fraction in (0., .1, .25, .5, .75, .9, 1.):
        x, y = sx+fraction*(ex-sx), sy+fraction*(ey-sy)
        entry = (x+side*offset*nx, y+side*offset*ny)
        exit = (x-side*offset*nx, y-side*offset*ny)
        if not clear_segment(grid, entry, exit):
            continue
        # Prefer endpoints with longer clear windows, then shorter travel.
        cost = math.dist(points[0], entry)+math.dist(exit, points[-1])
        cost += 2.*min(fraction, 1.-fraction)
        candidates.append((cost, entry, exit))
    if not candidates:
        return None
    _, entry, exit = min(candidates)
    return dict(track=track, entry=entry, exit=exit, bypass=False,
                yaw=math.atan2(exit[1]-entry[1], exit[0]-entry[0]))
