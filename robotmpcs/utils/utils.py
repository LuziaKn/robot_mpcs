import casadi as ca
import yaml
def diagSX(val, size):
    a = ca.SX(size, size)
    for i in range(size):
        a[i, i] = val[i]
    return a

def extractVariables(z, n, nx, nu, ns):
    q = z[0: n]
    qdot = z[n: nx]
    qddot = z[nx + ns : nx + ns + nu]
    return q, qdot, qddot

def get_velocity(z, n, nx):
    return  z[n: nx]


def point_to_plane(point: ca.SX, plane: ca.SX) -> ca.SX:
    distance = ca.fabs(ca.dot(plane[0:3], point) + plane[3]) / ca.norm_2(
        plane[0:3]
    )
    return distance

def parse_setup(setup_file: str):
    with open(setup_file, "r") as setup_stream:
        setup = yaml.safe_load(setup_stream)
    return setup