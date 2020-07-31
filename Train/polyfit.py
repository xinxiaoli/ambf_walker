from lib.GaitAnalysisToolkit.LearningTools.Trainer import TPGMMTrainer, GMMTrainer
from lib.GaitAnalysisToolkit.LearningTools.Runner import  GMMRunner, TPGMMRunner
from random import seed
from random import gauss
import numpy as np
import matplotlib.pyplot as plt
import numpy.polynomial.polynomial as poly

def coef(b, dt):

    A = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [1.0, dt, dt**2, dt**3],
                  [0, 1.0, 2*dt, 3*dt**2]])


    return np.linalg.pinv(A).dot(b)


# seed random number generator
seed(1)

b = np.array([ [-0.3], [0.0], [ -0.6  ], [0.0] ])
x = coef(b, 10)
fit = poly.Polynomial(x.flatten())
t = np.linspace(0,10,100)
y_prime = fit(t)
trajs = []
for i in xrange(10):
    y = y_prime + gauss(-0.01, 0.01)
    trajs.append(y)

trainer = TPGMMTrainer.TPGMMTrainer(trajs, "/home/nathanielgoldfarb/catkin_ws/src/ambf_walker/config/poly", 25, 0.01)
trainer.train()
print "hello"
runner = TPGMMRunner.TPGMMRunner("/home/nathanielgoldfarb/catkin_ws/src/ambf_walker/config/poly.pickle")
path = runner.run()


for p in trajs:
    plt.plot(p)

plt.plot(path, linewidth=4)
plt.show()