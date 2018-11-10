%matplotlib inline

import numpy
import numpy.linalg
import numpy.random
import matplotlib.pyplot as plt


G = 50.0  # гравитационная постоянная
collision_distance = 3.0  # всё-таки это не точки
model_delta_t = 0.01
time_to_model = 10
q=3   # кол-во масс-1

class MaterialPoint:
    """Материальная точка, движущаяся по двумерной плоскости"""
    
    def __init__(self, mass: 'float', position: 'numpy.array', velocity: 'numpy.array'):
        # Аннотации типов по желанию, но могут помочь IDE и компилятору, когда таковые имеются
        self.mass = mass
        self.position = position
        self.velocity = velocity
    
    @staticmethod
    def gravity_dencity(dist: 'float')-> 'float':
        if dist > collision_distance:
            return G / dist**2 
        else:
            return -G / dist  **2  # будем считать, что отскакивают точки друг от друга резко, но стараться не допускать этого
    
    def force_induced_by_other(self, other: 'MaterialPoint')-> 'numpy.array':
        delta_p = other.position - self.position
        distance = numpy.linalg.norm(delta_p)  # Евклидова норма (по теореме Пифагора)
        force_direction = delta_p / distance
        force = force_direction * self.mass * other.mass * MaterialPoint.gravity_dencity(distance)
        return force
    
    def advance(self):
        # print(self.position)
        self.position += self.velocity * model_delta_t

    def apply_force(self, force: 'numpy.array'):
        # print(force)
        self.velocity += force * model_delta_t / self.mass


centrum = MaterialPoint(500.0, numpy.array([0.0, 0.0]), numpy.array([0.0, 0.0]))
points=[]
masss=numpy.random.random (q)*10
positions=numpy.random.random((q,2))*50
velocitys=numpy.random.random((q,2))*20

for i in range (q):
    point_0 = MaterialPoint(masss[i], positions[i], velocitys[i])
    points.append(point_0)


def model_step():
    for i in range (q):
        points[i].apply_force(points[i].force_induced_by_other(centrum))
        points[i].advance()

xs=[]
ys=[]
for i in range(q):
    xs.append([])
    ys.append([])
print(len(ys))
    
for stepn in range(int(time_to_model / model_delta_t)):
    for i in range(q):
        xs[i].append(points[i].position[0])
        ys[i].append(points[i].position[1])
    model_step()

c = plt.Circle((0, 0), 2, color='b')
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.add_patch(c)

for i in range(q):
    plt.plot(xs[i],ys[i])
    

plt.show()
